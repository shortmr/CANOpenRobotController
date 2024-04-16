/**
 * @file JointM1.cpp
 * @author Yue Wen, Tim Haswell borrowing heavily from Vincent Crocher
 * @brief
 * @version 0.1
 * @date 2020-07-08
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "JointM1.h"

#include <iostream>

JointM1::JointM1(int jointID, double q_min, double q_max, short int sign_, double dq_min, double dq_max, double tau_min, double tau_max, KincoDrive *kincoDrive, const std::string& name): Joint(jointID, q_min, q_max, kincoDrive, name),
                                                                                                                                          sign(sign_), qMin(q_min), qMax(q_max), dqMin(dq_min), dqMax(dq_max), tauMin(tau_min), tauMax(tau_max){
    d2r = M_PI / 180.;
    r2d = 180. / M_PI;
    // Define unchanging unit conversion properties
    double round2radian = 2*M_PI;
    double correctionFactor = 1875./512;   //magic number
    encoderCounts = 10000;          //Encoder counts per turn
    reductionRatio = 69;            // Reduction ratio due to gear head, seems right, but not sure yet
    d2j_Pos = round2radian * 1 / (double)encoderCounts / reductionRatio;   // Drive to Joint unit conversion for position, in radian
    j2d_Pos = 1 * (double)encoderCounts * reductionRatio / round2radian;   // Joint to Drive unit conversion for position, in encoder count

    d2j_Vel = round2radian * 1 / 60. / (double)encoderCounts / reductionRatio * correctionFactor;   // Drive to Joint unit conversion for velocity, reading is encoder count per minutes
    j2d_Vel = 1 * 60. * (double)encoderCounts * reductionRatio/round2radian / correctionFactor;   // Joint to Drive unit conversion for velocity, command input is round per second

    Ipeak = 45.0;                   //Kinco FD123 peak current
    motorTorqueConstant = 0.132;    //SMC60S-0020 motor torque constant
    d2j_Trq = 1 / Ipeak / 1.414 * motorTorqueConstant * reductionRatio;   // Drive to Joint unit conversion for torque
    j2d_Trq = 1 * Ipeak * 1.414 / motorTorqueConstant / reductionRatio;   // Joint to Drive unit conversion for torque

    spdlog::debug("Joint ID {} Created", this->id);
}

JointM1::~JointM1() {
    delete drive;
}

bool JointM1::initNetwork() {
    spdlog::debug("JointM1::initNetwork()");
    return drive->init();
}

/***************************************************************************************/
/****************** implementation of virtual function of ActuatedJoint class **********/
/***************************************************************************************/
// convert from driver unit to joint unit for reading command
double JointM1::driveUnitToJointPosition(int driveValue)  {
    return d2j_Pos * (double)driveValue;
}

double JointM1::driveUnitToJointVelocity(int driveValue) {
    return d2j_Vel * driveValue;
}

double JointM1::driveUnitToJointTorque(int driveValue) {
    return d2j_Trq * driveValue;
}

// covert from joint unit to driver unit for control command
int JointM1::jointPositionToDriveUnit(double jointValue) {
//    DEBUG_OUT("jointPositionToDriveUnit " << count);
    return int(round(j2d_Pos * jointValue));
}

int JointM1::jointVelocityToDriveUnit(double jointValue) {
    return int(round(j2d_Vel * jointValue));
}

int JointM1::jointTorqueToDriveUnit(double jointValue) {
    return int(round(j2d_Trq * jointValue));
}

bool JointM1::updateValue() {
//    position = driveUnitToJointPosition(drive->getPos()) - q0;
    position = driveUnitToJointPosition(drive->getPos());
    velocity = driveUnitToJointVelocity(drive->getVel());
    torque = driveUnitToJointTorque(drive->getTorque());
    return true;
}

setMovementReturnCode_t JointM1::safetyCheck() {
    if(velocity > dqMax  ||  velocity < dqMin) {
        spdlog::debug("Velocity out of bound:  {}", velocity);
        return OUTSIDE_LIMITS;
    }
    if(torque > tauMax  ||  torque < tauMin) {
        spdlog::debug("Torque out of bound:  {}", torque);
        return OUTSIDE_LIMITS;
    }
    return SUCCESS;
}

/***************************************************************************************/
/****************** Check command safety range and send command to driver **************/
/***************************************************************************************/
// including position command, velocity command, torque command
setMovementReturnCode_t JointM1::setPosition(double qd) {
    if(calibrated) {
        if(sign*qd >= qMin  &&  sign*qd <= qMax) {
            return Joint::setPosition(qd);
        }
        else {
            spdlog::debug("Position out of bound: {}", r2d*sign*qd);
            return OUTSIDE_LIMITS;
        }
    }
    else {
        return NOT_CALIBRATED;
    }
}

setMovementReturnCode_t JointM1::setVelocity(double dqd) {
    //Position protection first only if calibrated
    if(calibrated) {
        if(sign*position <= qMin  &&  sign*dqd < 0) {
            dqd = 0;
        }
        if(sign*position >= qMax  &&  sign*dqd > 0) {
            dqd = 0;
        }
    }

    //Capped velocity
    if(sign*dqd>=dqMin && sign*dqd<=dqMax) {
        return Joint::setVelocity(dqd);
        return SUCCESS;

    }
    else {
        spdlog::debug("Velocity out of bound: {}", r2d*sign*dqd);
        return OUTSIDE_LIMITS;
    }
}

setMovementReturnCode_t JointM1::setTorque(double taud) {
    //Position protection first only if calibrated
    if(calibrated) {
        if(sign*position <= qMin  &&  sign*taud < 0) {
            taud = 0;
        }
        if(sign*position >= qMax  &&  sign*taud > 0) {
            taud = 0;
        }
    }
    //Capped torque
    if(sign*taud >= tauMin  &&  sign*taud <= tauMax) {
        return Joint::setTorque(taud);
    }
    else {
        spdlog::debug("Torque out of bound: {}", sign*taud);
        return OUTSIDE_LIMITS;
    }
}

void JointM1::errorMessage(setMovementReturnCode_t errorCode){
    switch(errorCode) {
        case SUCCESS:
            spdlog::debug(" ActuatedJoint::Success");
            break; //optional
        case OUTSIDE_LIMITS:
            spdlog::error(" ActuatedJoint::Outside of limitations");
            break; //optional
        case INCORRECT_MODE:
            spdlog::error(" ActuatedJoint::Incorrect mode");
            break; //optional
        case NOT_CALIBRATED:
            spdlog::error(" ActuatedJoint::Not calibrated");
            break; //optional
        default : //Optional
            spdlog::error(" ActuatedJoint::Unknown error");
    }
}

bool JointM1::setDigitalOut(int digital_out) {
    drive->setDigitalOut(digital_out);
    return true;
}

int JointM1::getDigitalIn() {
    return drive->getDigitalIn();
}

bool JointM1::checkCalibrationApplied() {
    calibrated = ((KincoDrive *)drive)->checkCalibrationApplied();
    if (calibrated) {
        q0 = 0.0;
    }
    return calibrated;
}

void JointM1::setPositionOffset(double offset) {
    ((KincoDrive *)drive)->setPositionOffset(jointPositionToDriveUnit(offset));
//    q0 = driveUnitToJointPosition(drive->getPos());
    q0 = 0.0;
    calibrated = true;
}

void JointM1::setSafetyPositionLimits(double min, double max) {
    qMin = min;
    qMax = max;
}
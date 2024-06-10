#include "RobotM1.h"

using namespace Eigen;

RobotM1::RobotM1(std::string robotName) : Robot(), calibrated(false), maxEndEffVel(2), maxEndEffForce(60), robotName_(robotName) {
    // Conversion factors between degrees and radians
    d2r = M_PI / 180.;
    r2d = 180. / M_PI;

    //Define the robot structure: each joint with limits and drive - TMH
    // JOINT 0 - the only joint in the case of M1
    max_speed(0) = 360; // {radians/s}
    tau_max(0) = 4.0 * 23;  // {Nm}
    LinkLengths(0) = 0.1;   // Link lengths used for kinematic models (in m)
    LinkMasses(0) = 0.5;    // Link masses used for gravity compensation (in kg)
    CoGLengths(0) = 0.08;    // Length along link(s) to the Center og Gravity
    Zero2GravityAngle(0) = 0;    // Angle from q=0 to the direction of gravitational force
    // g is the gravitational constant affecting that link, if its motion isn't subject to gravity set to 0
    g(0) = 9.81;  //Gravitational constant: remember to change it if using the robot on the Moon or another planet

    // Calibration configuration: posture in which the robot is when using the calibration procedure
    qCalibration(0) = 0 * d2r;
    tau_motor(0) = 0;
    q_filt(0) = 0;
    dq_filt(0) = 0;
    tau_s_filt(0) = 0;
    q_filt_pre(0) = 0;
    dq_filt_pre(0) = 0;
    tau_s_filt_pre(0) = 0;

    q_lim_ = Eigen::VectorXd::Zero(2);
    qp_lim_ = Eigen::VectorXd::Zero(2);
    tau_lim_ = Eigen::VectorXd::Zero(2);

    // Initializing the yaml parameters to zero
    m1Params.configFlag = true;
    m1Params.leftFlag = false;
    m1Params.wristFlag = false;
    m1Params.c0 = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.c1 = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.c2 = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.i_sin = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.i_cos = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.t_bias = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.force_sensor_scale_factor = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.force_sensor_offset = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.friction_ratio = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.weight_ratio = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.kp = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.kp_mod = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.ki = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.kd = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.vel_thresh = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.tau_thresh = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.sensor_cutoff_freq = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.motor_torque_cutoff_freq = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.tick_max = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.tracking_df = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.tracking_pf = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.passive_df = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.passive_pf = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.mvc_df = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.mvc_pf = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.muscle_count = Eigen::VectorXi::Zero(M1_NUM_JOINTS);

    initializeRobotParams(robotName_);

    // Set up the motor profile
    posControlMotorProfile.profileVelocity = 600.*512*10000/1875;
    posControlMotorProfile.profileAcceleration = 500.*65535*10000/4000000;
    posControlMotorProfile.profileDeceleration = 500.*65535*10000/4000000;

    initialiseJoints();
    initialiseInputs();

    tauCheck_ = true;
    checkNum_ = 0;
    tauDiff_ = 0.0;

    mode = 0;

    status = R_SUCCESS;
}

RobotM1::~RobotM1() {
    spdlog::debug(" Delete RobotM1 object begins");

    for (auto p : joints) {
        spdlog::debug(" Delete Joint ID: {}.", p->getId());
        delete p;
    }
    joints.clear();
    inputs.clear();
    delete keyboard;
    spdlog::debug("RobotM1 deleted");
}

bool RobotM1::initialiseJoints() {
    joints.push_back(new JointM1(0, min_pos(0), max_pos(0), limb_sign_, -max_speed(0), max_speed(0), -tau_max(0), tau_max(0), new KincoDrive(1), "q1"));
    return true;
}

bool RobotM1::initialiseNetwork() {
    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }
    //Give time to drives PDO initialisation
    //TODO: Parameterize the number of PDOs for situations like the one below
    //
    // spdlog::debug("...");
    // for (uint i = 0; i < 5; i++) {
    //     spdlog::debug(".");
    //     usleep(10000);
    // }
    return true;
}

bool RobotM1::initialiseInputs() {
    inputs.push_back(keyboard = new Keyboard());
    inputs.push_back(m1ForceSensor = new FourierForceSensor(17,
                                                            m1Params.force_sensor_scale_factor[0],
                                                            1,
                                                            m1Params.force_sensor_offset[0]));
    return true;
}

bool RobotM1::initializeRobotParams(std::string robotName) {

    // need to use address of base directory because when run with ROS, working directory is ~/.ros
    std::string baseDirectory = XSTR(BASE_DIRECTORY);
    std::string relativeFilePath = "/config/m1_params.yaml";

    YAML::Node params = YAML::LoadFile(baseDirectory + relativeFilePath);

    // if the robotName does not match with the name in m1_params.yaml
    if(!params[robotName]){
        spdlog::error("Parameters of {} couldn't be found in {} !", robotName, baseDirectory + relativeFilePath);
        spdlog::error("All parameters are zero !");
        return false;
    }

    if(params[robotName]["c0"].size() != M1_NUM_JOINTS || params[robotName]["c1"].size() != M1_NUM_JOINTS ||
       params[robotName]["c2"].size() != M1_NUM_JOINTS || params[robotName]["i_sin"].size() != M1_NUM_JOINTS ||
       params[robotName]["i_cos"].size() != M1_NUM_JOINTS || params[robotName]["t_bias"].size() != M1_NUM_JOINTS ||
       params[robotName]["force_sensor_scale_factor"].size() != M1_NUM_JOINTS ||
       params[robotName]["force_sensor_offset"].size() != M1_NUM_JOINTS ||
       params[robotName]["friction_ratio"].size() != M1_NUM_JOINTS || params[robotName]["weight_ratio"].size() != M1_NUM_JOINTS ||
       params[robotName]["kp"].size() != M1_NUM_JOINTS || params[robotName]["kp_mod"].size() != M1_NUM_JOINTS ||
       params[robotName]["ki"].size() != M1_NUM_JOINTS || params[robotName]["kd"].size() != M1_NUM_JOINTS ||
       params[robotName]["vel_thresh"].size() != M1_NUM_JOINTS || params[robotName]["tau_thresh"].size() != M1_NUM_JOINTS ||
       params[robotName]["sensor_cutoff_freq"].size() != M1_NUM_JOINTS ||
       params[robotName]["motor_torque_cutoff_freq"].size() != M1_NUM_JOINTS ||
       params[robotName]["tick_max"].size() != M1_NUM_JOINTS ||
       params[robotName]["tracking_df"].size() != M1_NUM_JOINTS || params[robotName]["tracking_pf"].size() != M1_NUM_JOINTS ||
       params[robotName]["passive_df"].size() != M1_NUM_JOINTS || params[robotName]["passive_pf"].size() != M1_NUM_JOINTS ||
       params[robotName]["mvc_df"].size() != M1_NUM_JOINTS || params[robotName]["mvc_pf"].size() != M1_NUM_JOINTS ||
       params[robotName]["muscle_count"].size() != M1_NUM_JOINTS) {

        spdlog::error("Parameter sizes are not consistent");
        spdlog::error("All parameters are zero !");

        return false;
    }

    // getting the parameters from the yaml file
    m1Params.configFlag = params["config_flag"].as<bool>();
    m1Params.leftFlag = params[robotName]["left"].as<bool>();
    m1Params.wristFlag = params[robotName]["wrist"].as<bool>();
    for(int i = 0; i<M1_NUM_JOINTS; i++){
        m1Params.c0[i] = params[robotName]["c0"][i].as<double>();
        m1Params.c1[i] = params[robotName]["c1"][i].as<double>();
        m1Params.c2[i] = params[robotName]["c2"][i].as<double>();
        m1Params.i_sin[i] = params[robotName]["i_sin"][i].as<double>();
        m1Params.i_cos[i] = params[robotName]["i_cos"][i].as<double>();
        m1Params.t_bias[i] = params[robotName]["t_bias"][i].as<double>();
        m1Params.force_sensor_scale_factor[i] = params[robotName]["force_sensor_scale_factor"][i].as<double>();
        m1Params.force_sensor_offset[i] = params[robotName]["force_sensor_offset"][i].as<double>();
        m1Params.friction_ratio[i] = params[robotName]["friction_ratio"][i].as<double>();
        m1Params.weight_ratio[i] = params[robotName]["weight_ratio"][i].as<double>();
        m1Params.kp[i] = params[robotName]["kp"][i].as<double>();
        m1Params.kp_mod[i] = params[robotName]["kp_mod"][i].as<double>();
        m1Params.ki[i] = params[robotName]["ki"][i].as<double>();
        m1Params.kd[i] = params[robotName]["kd"][i].as<double>();
        m1Params.vel_thresh[i] = params[robotName]["vel_thresh"][i].as<double>();
        m1Params.tau_thresh[i] = params[robotName]["tau_thresh"][i].as<double>();
        m1Params.sensor_cutoff_freq[i] = params[robotName]["sensor_cutoff_freq"][i].as<double>();
        m1Params.motor_torque_cutoff_freq[i] = params[robotName]["motor_torque_cutoff_freq"][i].as<double>();
        m1Params.tick_max[i] = params[robotName]["tick_max"][i].as<double>();
        m1Params.tracking_df[i] = params[robotName]["tracking_df"][i].as<double>();
        m1Params.tracking_pf[i] = params[robotName]["tracking_pf"][i].as<double>();
        m1Params.passive_df[i] = params[robotName]["passive_df"][i].as<double>();
        m1Params.passive_pf[i] = params[robotName]["passive_pf"][i].as<double>();
        m1Params.mvc_df[i] = params[robotName]["mvc_df"][i].as<double>();
        m1Params.mvc_pf[i] = params[robotName]["mvc_pf"][i].as<double>();
        m1Params.muscle_count[i] = params[robotName]["muscle_count"][i].as<int>();
    }

    // Set type of friction compensation
    hysteresisFlag_ = false;

    // Set whether device is for left or right side
    if (m1Params.leftFlag) {
        limb_sign_ = -1;
    }
    else {
        limb_sign_ = 1;
    }

    // Set whether device is for wrist or ankle
    if (m1Params.wristFlag) {
        max_pos(0) = 155 * d2r;
        min_pos(0) =  5 * d2r;
    }
    else {
        max_pos(0) = 130 * d2r;
        min_pos(0) =  10 * d2r;
    }

    i_sin_ = m1Params.i_sin[0];
    i_cos_ = m1Params.i_cos[0];
    t_bias_ = m1Params.t_bias[0];
    f_s_ = m1Params.c0[0];
    f_d_ = m1Params.c1[0];
    c2_ = m1Params.c2[0];
    q_lim_[0] = m1Params.tracking_df[0]*d2r; // radians
    q_lim_[1] = m1Params.tracking_pf[0]*d2r; // radians
    q_offset_ = 0.5*(q_lim_[0]+q_lim_[1]); // radians
    qp_lim_[0] = m1Params.passive_df[0]*d2r; // radians
    qp_lim_[1] = m1Params.passive_pf[0]*d2r; // radians
    tau_lim_[0] = m1Params.mvc_df[0]; // Nm
    tau_lim_[1] = m1Params.mvc_pf[0]; // Nm
    if (!m1Params.configFlag) {
        velThresh_ = m1Params.vel_thresh[0] * d2r;
        torqueThresh_ = m1Params.tau_thresh[0];
        setStaticFrictionFlag(0.0);
    }

    tau_offset_ = 0;
    f_s_upper_ = 0;
    f_s_lower_ = 0;
    f_d_up_ = 0;
    f_d_down_ = 0;
    f_s_theta1_ = 0;
    f_s_theta2_ = 0;
    return true;
}

RobotParameters RobotM1::sendRobotParams() {
    return m1Params;
}

bool RobotM1::stop() {
    std::cout << "Stopping M1 robot..." << std::endl;
    for (auto p : joints) {
        ((JointM1 *)p)->disable();
    }
    return true;
}

void RobotM1::applyCalibration() {
    for (uint i = 0; i < joints.size(); i++) {
        ((JointM1 *)joints[i])->setPositionOffset(qCalibration(i));
    }
    calibrated = true;
}

bool RobotM1::calibrateForceSensors() {
    if(m1ForceSensor->sendInternalCalibrateSDOMessage()){
        spdlog::info("[RobotM1::calibrateForceSensors]: Force sensor zeroing completed. Sensor value is {}", m1ForceSensor->getSensorValue());
        return true;
    } else {
        spdlog::info("[RobotM1::calibrateForceSensors]: Zeroing failed.");
        return false;
    }
}

void RobotM1::updateRobot() {
    Robot::updateRobot();   // Trigger RT data update at the joint level
    // Gather joint data at the Robot level
    //TODO: we should probably do this with all PDO data, if we are setting it up
    // to be delivered in real-time we should make it available and use it or stop
    // sending it in real-time to conserve bandwidth. It would also be good to break
    // down the status word into a vector of booleans and have descriptive indices to
    // be able to clearly access the bits in a meaningful and readable way. -TMH

    for(uint i = 0; i < nJoints; i++) {
        q(i) = limb_sign_*((JointM1 *)joints[i])->getPosition();
        dq(i) = limb_sign_*((JointM1 *)joints[i])->getVelocity();
        tau(i) = limb_sign_*((JointM1 *)joints[i])->getTorque();
        tau_s(i) = limb_sign_*m1ForceSensor[i].getForce();

        // compensate weight for torque sensor measurement
        tau_s(i) =  tau_s(i) + i_sin_*sin(q(i)+t_bias_) + i_cos_*cos(q(i)+t_bias_);
    }

    // filter interaction torque measurements
    filter_tau_s(alpha_sensor_);

    // check for error in torque/position/velocity measurements and calibration
    if (tauCheck_) {
        if (checkNum_ == 0) {
            tau_prev_ = tau(0);
        }
        else {
            tauDiff_ = (tauDiff_+abs(tau(0)-tau_prev_));

            if (checkNum_ == 333) {
                tauCheck_ = false;
                if (tauDiff_/333 <= 0.001) {
                    std::cout << "M1: Joint " << 0 << " ERROR (sensor readings with mean diff: " << tauDiff_/333 << ")" << std::endl;
                }
                else {
                    std::cout << "M1: Joint " << 0 << " PASS (sensor readings with mean diff: " << tauDiff_/333 << ")" << std::endl;
                }

                if (checkCalibrationApplied()){
                    std::cout << "M1: Joint " << 0 << " is calibrated" << std::endl;
                }
                else {
                    std::cout << "M1: Joint " << 0 << " is NOT calibrated" << std::endl;
                }
            }
        }
        checkNum_ += 1;
    }

    if (safetyCheck() != SUCCESS) {
        status = R_OUTSIDE_LIMITS;
        stop();
    }
}

bool RobotM1::setDigitalOut(int digital_out) {
    ((JointM1 *)joints[0])->setDigitalOut(digital_out);
    return true;
}

int RobotM1::getDigitalIn() {
    return ((JointM1 *)joints[0])->getDigitalIn();
}

bool RobotM1::checkCalibrationApplied() {
    calibrated = ((JointM1 *)joints[0])->checkCalibrationApplied();
    return calibrated;
}

setMovementReturnCode_t RobotM1::safetyCheck() {
    //End-effector safeties if calibrated
    for (uint i = 0; i < nJoints; i++) {    // Error found, YW
        if (((JointM1 *)joints[i])->safetyCheck() != SUCCESS) {
            std::cout /*cerr is banned*/ << "M1: Joint " << i << " safety triggered!" << std::endl;
            return OUTSIDE_LIMITS;
        }
    }
    return SUCCESS;
}

void RobotM1::printStatus() {
    std::cout << std::setprecision(3) << std::fixed;
//    std::cout << "X=[ " << getEndEffPos().transpose() << " ]\t";
//    std::cout << "dX=[ " << getEndEffVel().transpose() << " ]\t";
//    std::cout << "F=[ " << getEndEffFor().transpose() << " ]\t";
    std::cout << std::endl;
}

void RobotM1::printJointStatus() {
    std::cout << std::setprecision(3) << std::fixed;
    std::cout << "q=[ " << getJointPos().transpose() << " ]\t";
    std::cout << "dq=[ " << getJointVel().transpose() << " ]\t";
    std::cout << "tau=[ " << getJointTor().transpose() << " ]\t";
    std::cout << "tau_s=[ " << getJointTor_s().transpose() << " ]\t";
    std::cout << "{";
    for (auto joint : joints)
        std::cout << "0x" << std::hex << ((JointM1 *)joint)->getDriveStatus() << "; ";
    std::cout << "}" << std::endl;
}

bool RobotM1::initMonitoring() {
    spdlog::debug("Initialising monitoring all joints.");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM1 *)p)->setMode(CM_POSITION_CONTROL, posControlMotorProfile) != CM_POSITION_CONTROL) {
            // Something bad happened if we are here
            spdlog::debug("Something bad happened.");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
//        ((JointM1 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM1 *)p)->disable();
    }
    return returnValue;
}

bool RobotM1::initPositionControl() {
    spdlog::debug("Initialising Position Control on all joints.");
    bool returnValue = true;

    for (auto p : joints) {
        if (((JointM1 *)p)->setMode(CM_POSITION_CONTROL, posControlMotorProfile) != CM_POSITION_CONTROL) {
            // Something bad happened if we are here
            spdlog::debug("Something bad happened.");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
//        ((JointM1 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
   usleep(2000);
   for (auto p : joints) {
       ((JointM1 *)p)->enable();
   }
    mode = 1;
    return returnValue;
}

bool RobotM1::initVelocityControl() {
    spdlog::debug("Initialising Velocity Control on all joints.");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM1 *)p)->setMode(CM_VELOCITY_CONTROL, posControlMotorProfile) != CM_VELOCITY_CONTROL) {
            // Something bad happened if we are here
            spdlog::debug("Something bad happened.");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM1 *)p)->readyToSwitchOn();
    }
    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM1 *)p)->enable();
    }
    mode = 2;
    return returnValue;
}

bool RobotM1::initTorqueControl() {
    spdlog::debug("Initialising Torque Control on all joints.");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM1 *)p)->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            // Something bad happened if we are here
            spdlog::debug("Something bad happened.");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM1 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM1 *)p)->enable();
    }
    mode = 3;
    return returnValue;
}

setMovementReturnCode_t RobotM1::applyPosition(JointVec positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((JointM1 *)p)->setPosition(positions(i));
        if (setPosCode == INCORRECT_MODE) {
            std::cout << "Joint " << p->getId() << ": is not in Position Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << ": " << std::endl;
            ((JointM1 *)p)->errorMessage(setPosCode);
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

setMovementReturnCode_t RobotM1::applyVelocity(JointVec velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
//        std::cout << "Joint velocity 2: " << velocities(0) << std::endl;
        setMovementReturnCode_t setVelCode = ((JointM1 *)p)->setVelocity(velocities(i));
        if (setVelCode == INCORRECT_MODE) {
            std::cout << "Joint " << p->getId() << ": is not in Velocity Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setVelCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << " velocity : " << std::endl;
            ((JointM1 *)p)->errorMessage(setVelCode);
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

setMovementReturnCode_t RobotM1::applyTorque(JointVec torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setTorCode = ((JointM1 *)p)->setTorque(torques(i));
        if (setTorCode == INCORRECT_MODE) {
            std::cout << "Joint " << p->getId() << ": is not in Torque Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setTorCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << ": Unknown Error " << std::endl;
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

EndEffVec RobotM1::directKinematic(JointVec q_given) {
    Vector2d X;

    X(0) = LinkLengths(0) * cos(q_given(0));
    X(1) = LinkLengths(0) * sin(q_given(0));

    return X;
}

JointVec RobotM1::inverseKinematic(EndEffVec X) {
    // Calculate joint angle from (X,Y)
    JointVec q_exp;
    q_exp(0) = -atan2(X(1), -X(0));
    return q_exp;
}

JacMtx RobotM1::J() {
    JacMtx J;

    //Jacobian matrix elements
    //TODO: Check Jacobian calculations for M1 - what angle will be considered 0
    J(0) = LinkLengths(0) * sin(q(0));
    J(1) = LinkLengths(0) * cos(q(0));

    return J;
}

JointVec RobotM1::calculateGravityTorques() {
    JointVec tau_g;

    //Calculate gravitational torques
    tau_g(0) = LinkMasses(0) * g(0) * CoGLengths(0) * sin(q(0) - Zero2GravityAngle(0));

    return tau_g;
}

JointVec RobotM1::getJointPos() {
    return q*r2d;
}

JointVec RobotM1::getJointVel() {
    return dq*r2d;
}

JointVec RobotM1::getJointTor() {
    return tau;
}

JointVec & RobotM1::getPosition() {
    return q;
}

JointVec & RobotM1::getVelocity() {
    return dq;
}

JointVec & RobotM1::getTorque() {
    return tau;
}

void RobotM1::filter_q(double alpha_q){
    for(int i = 0; i<M1_NUM_JOINTS; i++) {
        q_filt(i) = alpha_q*q(i)+(1-alpha_q)*q_filt_pre(i);
        q_filt_pre(i) = q_filt(i);
    }
}

void RobotM1::filter_dq(double alpha_dq){
    for(int i = 0; i<M1_NUM_JOINTS; i++) {
        dq_filt(i) = alpha_dq*dq(i)+(1-alpha_dq)*dq_filt_pre(i);
        dq_filt_pre(i) = dq_filt(i);
    }
}

void RobotM1::filter_tau_s(double alpha_tau_s){
    for(int i = 0; i<M1_NUM_JOINTS; i++) {
        tau_s_filt(i) = alpha_tau_s*tau_s(i)+(1-alpha_tau_s)*tau_s_filt_pre(i);
        tau_s_filt_pre(i) = tau_s_filt(i);
    }
}

JointVec& RobotM1::getJointTor_s() {
    return tau_s;
}

JointVec& RobotM1::getJointTor_s_filt() {
    return tau_s_filt;
}

setMovementReturnCode_t RobotM1::setJointPos(JointVec pos_d) {
    // flip sign and return if device is for left limb
    for(int i = 0; i<M1_NUM_JOINTS; i++) {
        pos_d(i) = limb_sign_*pos_d(i);
    }
    return applyPosition(pos_d*d2r);
}

setMovementReturnCode_t RobotM1::setJointVel(JointVec vel_d) {
    // flip sign and return if device is for left limb
    for(int i = 0; i<M1_NUM_JOINTS; i++) {
        vel_d(i) = limb_sign_*vel_d(i);
    }
    return applyVelocity(vel_d*d2r);
}

setMovementReturnCode_t RobotM1::setJointTor(JointVec tor_d) {
    // flip sign and return if device is for left limb
    for(int i = 0; i<M1_NUM_JOINTS; i++) {
        tor_d(i) = limb_sign_*tor_d(i);
        tau_motor(i) = tor_d(i);
    }
    return applyTorque(tor_d);
}

setMovementReturnCode_t RobotM1::setJointTor_comp(JointVec tor, double fRatio, double wRatio) {
    double tor_ff;
    double tor_friction;
    double vel = dq(0);
    double pos = q(0);

    if (!hysteresisFlag_) {
        // original implementation
        if (!staticFrictionFlag_) {
            // Feedforward wrist compensation (viscous friction only)
            tor_ff = fRatio*(f_d_*vel) +
                     wRatio*(i_sin_*sin(pos+t_bias_) + i_cos_*cos(pos+t_bias_));
        }
        else {
            // Feedforward ankle compensation (linearized static friction, weight and viscous friction)
            if(abs(vel)<velThresh_)
            {
                double slowCoef = f_s_/velThresh_; // linear region for static friction
                tor_ff = fRatio*(slowCoef*vel) +
                         wRatio*(i_sin_*sin(pos+t_bias_) + i_cos_*cos(pos+t_bias_));
            } else {
                tor_ff = fRatio*(f_s_*sign(vel) + f_d_*vel + c2_*sqrt(abs(vel))*sign(vel)) +
                         wRatio*(i_sin_*sin(pos+t_bias_) + i_cos_*cos(pos+t_bias_));
            }
        }
    }
    else {
        // lorenzo hysteresis
        if (vel >= 0) {
            f_s_theta2_ = 1;
        }
        else {
            f_s_theta2_ = 0;
        }
        if(abs(vel)<velThresh_) {
            double f_s_pos = ((f_s_upper_ - f_s_lower_) / velThresh_) * vel + f_s_lower_;
            double f_s_neg = ((f_s_upper_ - f_s_lower_) / velThresh_) * vel + f_s_upper_;
            double f_s_up = f_s_upper_ * f_s_theta2_ + f_s_neg * (1 - f_s_theta2_);
            double f_s_down = f_s_pos * f_s_theta2_ + f_s_lower_ * (1 - f_s_theta2_);
            tor_friction = f_s_up * (1 - f_s_theta1_) + f_s_down * f_s_theta1_;
        } else {
            f_s_theta1_ = f_s_theta2_;
            double f_d_up = f_d_up_ * (vel - velThresh_) + f_s_upper_;
            double f_d_down = f_d_down_ * (vel + velThresh_) + f_s_lower_;
            tor_friction = f_d_up * f_s_theta2_ + f_d_down * (1 - f_s_theta2_);
        }
        tor_ff = fRatio*tor_friction + wRatio*(i_sin_ * sin(pos + t_bias_) + i_cos_ * cos(pos + t_bias_));
    }

    tor(0) = tor(0) + tor_ff;

    // filter command signal
    filteredMotorTorqueCommand_ = alpha_motor_torque_*tor(0)+(1-alpha_motor_torque_)*previousFilteredTorqueCommand_;
    previousFilteredTorqueCommand_ = filteredMotorTorqueCommand_;

    JointVec finalMotorTorqueCommand(M1_NUM_JOINTS);
    finalMotorTorqueCommand(0) = filteredMotorTorqueCommand_;

    return setJointTor(finalMotorTorqueCommand);
}

std::string & RobotM1::getRobotName() {
    return robotName_;
}

void RobotM1::setControlFreq(double controlFreq) {
    controlFreq_ = controlFreq;
}

void RobotM1::setVelThresh(double velThresh) {
    if (m1Params.configFlag) {
        velThresh_ = velThresh * d2r;
    }
}

void RobotM1::setHysteresisFrictionParams(double f_s, double f_d) {
    f_s_upper_ = f_s;
    f_s_lower_ = -1*f_s; // flip sign of lower static friction
    f_d_up_ = f_d;
    f_d_down_ = f_d;
}

void RobotM1::setTorqueThresh(double torqueThresh) {
    if (m1Params.configFlag) {
        torqueThresh_ = torqueThresh;
    }
}

void RobotM1::setMotorTorqueCutOff(double cutOff) {
    if (m1Params.configFlag) {
        motorTorqueCutOff_ = cutOff;
    }
    else {
        motorTorqueCutOff_ = m1Params.motor_torque_cutoff_freq[0];
    }
    alpha_motor_torque_ = (2*M_PI*motorTorqueCutOff_/controlFreq_)/(2*M_PI*motorTorqueCutOff_/controlFreq_+1);
}

void RobotM1::setSensorCutOff(double cutOff) {
    if (m1Params.configFlag) {
        sensorCutOff_ = cutOff;
    }
    else {
        sensorCutOff_ = m1Params.sensor_cutoff_freq[0];
    }
    alpha_sensor_ = (2*M_PI*sensorCutOff_/controlFreq_)/(2*M_PI*sensorCutOff_/controlFreq_+1);
}

void RobotM1::setTorqueOffset(double tau_filt) {
    tau_offset_ = tau_filt;
    std::cout << std::endl;
    std::cout << std::setprecision(3) << std::fixed << "  torque_offset: [" << tau_filt << "]" << std::endl;
    std::cout << std::endl;
}

void RobotM1::setMaxTorques(double tau_df, double tau_pf) {
    tau_lim_[0] = abs(tau_df);
    tau_lim_[1] = abs(tau_pf);

    std::cout << std::endl;
    std::cout << std::setprecision(3) << std::fixed << "  mvc_df: [" << tau_df << "] # MVCT in dorsiflexion [Nm]" << std::endl;
    std::cout << std::setprecision(3) << std::fixed << "  mvc_pf: [" << tau_pf << "] # MVCT in plantarflexion [Nm]" << std::endl;
    std::cout << std::endl;
}

void RobotM1::setMaxActiveAngles(double q_df, double q_pf) {
    q_lim_[0] = q_df*d2r;
    q_lim_[1] = q_pf*d2r;
    q_offset_ = 0.5*(q_df+q_pf)*d2r;

    std::cout << std::endl;
    std::cout << std::setprecision(3) << std::fixed << "  tracking_df: [" << q_df  << "] # max range of motion angle [deg]" << std::endl;
    std::cout << std::setprecision(3) << std::fixed << "  tracking_pf: [" << q_pf  << "] # min range of motion angle [deg]" << std::endl;
    std::cout << std::endl;
}

void RobotM1::setMaxPassiveAngles(double q_df, double q_pf) {
    qp_lim_[0] = q_df*d2r;
    qp_lim_[1] = q_pf*d2r;

    std::cout << std::endl;
    std::cout << std::setprecision(3) << std::fixed << "  passive_df: [" << q_df  << "] # max passive range of motion angle [deg]" << std::endl;
    std::cout << std::setprecision(3) << std::fixed << "  passive_pf: [" << q_pf  << "] # min passive range of motion angle [deg]" << std::endl;
    std::cout << std::endl;
}

double & RobotM1::getPositionOffset() {
    return q_offset_;
}

double & RobotM1::getTorqueOffset() {
    return tau_offset_;
}

Eigen::VectorXd & RobotM1::getPositionLimits(int type) {
    //active (0) or passive (1)
    if (type == 0) {
        return q_lim_;
    }
    else if (type == 1) {
        return qp_lim_;
    }
}

Eigen::VectorXd & RobotM1::getTorqueLimits() {
    return tau_lim_;
}

void RobotM1::setStaticFrictionFlag(double multiplier) {
    double m;
    if (m1Params.configFlag) {
        m = multiplier;
    }
    else {
        m = m1Params.kp_mod[0];
    }

    // Add static friction compensation if proportional multiplier is disabled
    if (m == 0.0) {
        staticFrictionFlag_ = true;
    } else {
        staticFrictionFlag_ = false;
    }
}

void RobotM1::disableJointPositionSafety() {
    ((JointM1 *)joints[0])->setSafetyPositionLimits(-100, 100);
}

void RobotM1::enableJointPositionSafety() {
    ((JointM1 *)joints[0])->setSafetyPositionLimits(min_pos(0), max_pos(0));
}

short RobotM1::sign(double val) { return (val > 0) ? 1 : ((val < 0) ? -1 : 0); }
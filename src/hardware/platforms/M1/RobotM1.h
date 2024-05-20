
/**
 *
 * \file RobotM1.h
 * \author Tim Haswell borrowing heavily from Vincent Crocher
 * \version 0.1
 * \date 2020-07-08
 * \copyright Copyright (c) 2020
 *
 * \brief  The<code> RobotM1</ code> class represents an M1 Robot.
 *
 */

#ifndef RobotM1_H_INCLUDED
#define RobotM1_H_INCLUDED

#include <map>
#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <cmath>

#include "JointM1.h"
#include "Keyboard.h"
#include "Joystick.h"
#include "Robot.h"
#include "FourierForceSensor.h"
#include "KincoDrive.h"

// yaml-parser
#include <fstream>
#include "yaml-cpp/yaml.h"

// These are used to access the MACRO: BASE_DIRECTORY
#define XSTR(x) STR(x)
#define STR(x) #x

#define M1_NUM_JOINTS 1
#define M1_NUM_INTERACTION 1

//typedef unsigned int uint;

// System description constants
static const uint nJoints = 1;  // Number of joints in the system
static const uint nEndEff = 2;  // Number of dimensions for the end effector data
typedef Eigen::Matrix<double, nJoints, 1> JointVec;
typedef Eigen::Matrix<double, nEndEff, 1> EndEffVec;
typedef Eigen::Matrix<double, nJoints, nEndEff> JacMtx;

struct RobotParameters {
    bool configFlag; // flag for using dynamic reconfigure
    bool leftFlag;
    bool wristFlag;
    Eigen::VectorXd c0; // coulomb friction const of joint [Nm]
    Eigen::VectorXd c1; // viscous fric constant of joint [Nm/s]
    Eigen::VectorXd c2; // friction square root term
    Eigen::VectorXd i_sin; // inertia for sine term
    Eigen::VectorXd i_cos; // inertia for cosine term
    Eigen::VectorXd t_bias; // theta bias from vertical axis [rad]
    Eigen::VectorXd force_sensor_scale_factor; // scale factor for force calibration
    Eigen::VectorXd force_sensor_offset; // offset for torque sensor
    Eigen::VectorXd friction_ratio; // scale factor for feedforward compensation
    Eigen::VectorXd weight_ratio; // scale factor for feedforward compensation
    Eigen::VectorXd kp; // proportional term of PID controller
    Eigen::VectorXd kp_mod; // multiplier for proportional term of PID controller
    Eigen::VectorXd ki; // integral term of PID controller
    Eigen::VectorXd kd; // derivative term of PID controller
    Eigen::VectorXd vel_thresh; // velocity threshold for implementing dynamic friction [deg/s]
    Eigen::VectorXd tau_thresh; // torque threshold for implementing dynamic friction [deg/s]
    Eigen::VectorXd sensor_cutoff_freq; // cutoff frequency for interaction torque, position and velocity filter [Hz]
    Eigen::VectorXd motor_torque_cutoff_freq; // cutoff frequency for motor torque command filter [Hz]
    Eigen::VectorXd tick_max; // counter for integral term reset [s]
    Eigen::VectorXd tracking_df; // range of motion [deg]
    Eigen::VectorXd tracking_pf; // range of motion [deg]
    Eigen::VectorXd passive_df; // passive range of motion [deg]
    Eigen::VectorXd passive_pf; // passive range of motion [deg]
    Eigen::VectorXd mvc_df; // maximum dorsiflexion torque [Nm]
    Eigen::VectorXd mvc_pf; // maximum plantarflexion torque [Nm]
    Eigen::VectorXi muscle_count; // number of EMG channels
};

/**
 * An enum type.
 * Constants representing the Drives State
 */
enum RobotState{
    R_SUCCESS = 1,
    R_OUTSIDE_LIMITS = -1,
    R_UNKNOWN_ERROR = -100
};

/**
     * \todo Load in parameters and dictionary entries from JSON file.
     *
     */

/**
 * \brief Implementation of the M1 robot class, representing an M1 using 1 JointM1 (and so Kinco drive).
 * model reference:
 */
class RobotM1 : public Robot {
   private:
    /**
     * \brief motor drive position control profile parameters, user defined.
     *
     */
    motorProfile posControlMotorProfile{2000000, 80000, 80000};

    RobotParameters m1Params;

    JointVec LinkLengths;   // Link lengths used for kinematic models (in m)
    JointVec LinkMasses;    // Link masses used for gravity compensation (in kg)
    JointVec CoGLengths;    // Length along link(s) to the Center og Gravity
    JointVec Zero2GravityAngle;    // Angle from q=0 to the direction of gravitational force
    JointVec g;  //Gravitational constant: remember to change it if using the robot on the Moon or another planet
    JointVec max_speed; // {radians}
    JointVec tau_max;  // {Nm}
    JointVec min_pos; // {radians}
    JointVec max_pos; // {radians}
    /*!< Conversion factors between degrees and radians */
    double d2r, r2d;

    // Storage variables for real-time updated values from CANopen
    JointVec q, dq, tau, tau_s;
    JointVec q_filt, dq_filt, tau_s_filt;
    JointVec q_filt_pre, dq_filt_pre, tau_s_filt_pre;

    JointVec qCalibration;  // Calibration configuration: posture in which the robot is when using the calibration procedure

    bool calibrated;
    double maxEndEffVel; /*!< Maximal end-effector allowable velocity. Used in checkSafety when robot is calibrated.*/
    double maxEndEffForce; /*!< Maximal end-effector allowable force. Used in checkSafety when robot is calibrated. */

    std::string robotName_;

    double velThresh_, torqueThresh_;

    double alpha_sensor_, alpha_motor_torque_;

    double f_s_upper_, f_s_lower_, f_d_up_, f_d_down_, f_s_theta1_, f_s_theta2_;

    double filteredMotorTorqueCommand_, previousFilteredTorqueCommand_;

    double motorTorqueCutOff_, sensorCutOff_;

    double controlFreq_;

    double i_sin_, i_cos_;

    bool hysteresisFlag_;

    double f_s_, f_d_, c2_;

    bool initializeRobotParams(std::string robotName);

    short int sign(double val);

    double tau_offset_, tau_df_, tau_pf_;
    double q_offset_, q_df_, q_pf_;

    Eigen::VectorXd q_lim_;
    Eigen::VectorXd qp_lim_;
    Eigen::VectorXd tau_lim_;

public:
    /**
      * \brief Default <code>RobotM1</code> constructor.
      * Initialize memory for the <code>Joint</code> + sensor.
      * Load in parameters to  <code>TrajectoryGenerator.</code>.
      */
    RobotM1(std::string robotName);
    ~RobotM1();
    JointVec tau_motor;
    Keyboard *keyboard;
    Joystick *joystick;
    FourierForceSensor *m1ForceSensor;
    RobotState status;
    int mode;
    double stim_df_, stim_pf_;
    bool stim_calib_;

    double t_bias_;

    JointVec tau_spring;

    bool initMonitoring();
    /**
       * \brief Initializes joint to position control mode.
       *
       * \return true If joint is successfully configured
       * \return false  If joint fails the configuration
       */
    bool initPositionControl();

    /**
       * \brief Initialises joint to velocity control mode.
       *
       * \return true If joint is successfully configured
       * \return false  If joint fails the configuration
       */
    bool initVelocityControl();

    /**
       * \brief Initializes joint to torque control mode.
       *
       * \return true If joint is successfully configured
       * \return false  If joint fails the configuration
       */
    bool initTorqueControl();

    /**
       * \brief Send a stop command to joint drive.
       *
       * \return true If joint is stopped
       * \return false  Otherwise
       */
    bool stop();

    /**
    * \brief Set the target position for the joint
    *
    * \param positions a target position - applicable for the actuated joint
    * \return MovementCode representing success or failure of the application
    */
    setMovementReturnCode_t applyPosition(JointVec positions);

    /**
    * \brief Set the target velocity for the joint
    *
    * \param velocities a target velocity - applicable for the actuated joint
    * \return MovementCode representing success or failure of the application
    */
    setMovementReturnCode_t applyVelocity(JointVec velocities);

    /**
    * \brief Set the target torque for the joint
    *
    * \param torques a target torque - applicable for the actuated joint
    * \return MovementCode representing success or failure of the application
    */
    setMovementReturnCode_t applyTorque(JointVec torques);

    /**
    * \brief Apply current configuration as calibration configuration using qcalibration such that:
    *  q=qcalibration in current configuration.
    */
    void applyCalibration();
    /**
    * \brief Send internal calibration command to torque sensor
    *
    */
    bool calibrateForceSensors();

    bool isCalibrated() {return calibrated;}
    void decalibrate() {calibrated = false;}


    /**
       * \brief Implementation of Pure Virtual function from <code>Robot</code> Base class.
       * Create designed <code>Joint</code> and <code>Driver</code> objects and load into
       * Robot joint vector.
       */
    bool initialiseJoints();
    /**
       * \brief Implementation of Pure Virtual function from <code>Robot</code> Base class.
       * Initialize each <code>Drive</code> Objects underlying CANOpen Networking.

      */
    bool initialiseNetwork();
    /**
       * \brief Implementation of Pure Virtual function from <code>Robot</code> Base class.
       * Initialize each <code>Input</code> Object.

      */
    bool initialiseInputs();
    /**
       * \brief update current state of the robot, including input and output devices.
       * Overloaded Method from the Robot Class.
       * Example. for a keyboard input this would poll the keyboard for any button presses at this moment in time.
       */
    void updateRobot();

    /**
       * Writes the desired digital out value to the drive
       *
       * \return true if called
       */
    bool setDigitalOut(int digital_out);

    /**
           * Returns the value of digital IN
           * \return Digital in state from the motor drive
           */
    virtual int getDigitalIn();

    bool checkCalibrationApplied();

    /**
       * Returns the value of vertical bias from parameter list
       * \return Angle (degrees) between lower joint limit and vertical pose for M1
       */
    double getVerticalBias();

    /**
     * \brief Check if current end effector force and velocities are within limits (if calibrated, otherwise
     *  check that joints velocity and torque are within limits).
     *
     * \return OUTSIDE_LIMITS if outside the limits (!), SUCCESS otherwise
     */
    setMovementReturnCode_t safetyCheck();


    /**
     * \brief get the name of the robot that is obtained from node name
     *
     * \return std::string name of the robot
     */
    std::string & getRobotName();

    RobotParameters sendRobotParams();

    double limb_sign_; // left or right limb
    bool staticFrictionFlag_;

    // check for error resulting in constant torque readings
    bool tauCheck_;
    int checkNum_;
    double tauDiff_;
    double tau_prev_;

    void setControlFreq(double controlFreq);
    void setVelThresh(double velThresh);
    void setHysteresisFrictionParams(double f_s, double f_d);
    void setTorqueThresh(double torqueThresh);
    void setMotorTorqueCutOff(double cutOff);
    void setSensorCutOff(double cutOff);
    void setMaxTorques(double tau_df, double tau_pf);
    void setMaxActiveAngles(double q_df, double q_pf);
    void setMaxPassiveAngles(double q_df, double q_pf);
    void setStaticFrictionFlag(double multiplier);
    void setStimAmplitude(double stim_df, double stim_pf);
    void setStimCalibrate(bool stim_calib);
    void setTorqueOffset(double tau_filt);

    void disableJointPositionSafety();
    void enableJointPositionSafety();

    void printStatus();
    void printJointStatus();

    JacMtx J();
    EndEffVec directKinematic(JointVec q);
    JointVec inverseKinematic(EndEffVec X);
    JointVec calculateGravityTorques();

    JointVec getJointPos();
    JointVec getJointVel();
    JointVec getJointTor();

    JointVec& getJointTor_s();
    JointVec& getJointTor_s_filt();
    JointVec& getJointTor_cmd();
    JointVec& getPosition();
    JointVec& getVelocity();
    JointVec& getTorque();

    double& getPositionOffset();
    double& getTorqueOffset();
    Eigen::VectorXd& getPositionLimits(int type);
    Eigen::VectorXd& getTorqueLimits();

    void filter_q(double alpha_q);
    void filter_dq(double alpha_dq);
    void filter_tau_s(double alpha_tau_s);
//    EndEffVec getEndEffPos();
//    EndEffVec getEndEffVel();
//    EndEffVec getEndEffFor();

    setMovementReturnCode_t setJointPos(JointVec pos);
    setMovementReturnCode_t setJointVel(JointVec vel);
    setMovementReturnCode_t setJointTor(JointVec tor);
    setMovementReturnCode_t setJointTor_comp(JointVec tor, double frictionRatio, double weightRatio);
};
#endif /*RobotM1_H*/

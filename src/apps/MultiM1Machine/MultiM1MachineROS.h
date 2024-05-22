/**
 * /file MultiM1MachineROS.h
 * /author Emek Baris Kucuktabak
 * /brief ROS part of the MultiM1Machine
 * /version 0.1
 * /date 2020-11-03
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef SRC_MultiM1MachineROS_H
#define SRC_MultiM1MachineROS_H

// msg types
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include "RobotM1.h"
#include "ros/ros.h"  // This state machine requires ROS
#include <CORC/JointScaled32.h>
#include <CORC/InteractionMode.h>
#include <CORC/SetMVC.h>

#include "trigno/TrignoMultiEMG.h"
#include "trigno/TrignoMultiIMU.h"
#include <trigno_msgs/trignoMultiIMU.h>
#include <trigno_msgs/trignoMultiEMG.h>
#include <trigno_msgs/trignoIMU.h>
#include <trigno_msgs/trignoEMG.h>

constexpr int N_EMG = 4;
constexpr int N_IMU = 4;

class MultiM1MachineROS {
public:
    MultiM1MachineROS(RobotM1 *robot, TrignoMultiEMG *trignoMultiEMG, TrignoMultiIMU *trignoMultiIMU);
    ~MultiM1MachineROS();

    void update(double time);
    void publishJointStates(void);
    void publishInteractionForces(void);
    void publishInteractionScaled(void);
    void publishJointScaled(void);
    void publishTrignoFilteredEMGs();
    void initialize();
    void setNodeHandle(ros::NodeHandle& nodeHandle);

    ros::NodeHandle* nodeHandle_;

    Eigen::VectorXd jointPositionCommand_, jointVelocityCommand_, jointTorqueCommand_;
    Eigen::VectorXd interactionTorqueCommand_;
    Eigen::VectorXd prbsPositionCommand_;

    Eigen::VectorXd jointPositionScaled_;
    Eigen::VectorXd jointTorqueScaled_;

    int interactionMode_;
    int referenceLimb_;

private:
    // Subscriber and callback func for joint command subscription
    ros::Subscriber jointCommandSubscriber_;
    void jointCommandCallback(const sensor_msgs::JointState &msg);

    // Subscriber and callback func for interaction torque subscription
    ros::Subscriber interactionTorqueCommandSubscriber_;
    void interactionTorqueCommandCallback(const std_msgs::Float64MultiArray &msg);

    // Subscriber and callback func prbs signal
    ros::Subscriber prbsCommandSubscriber_;
    void prbsCommandCallback(const geometry_msgs::Vector3 &msg);

    // Subscriber and callback function for interaction mode
    ros::Subscriber interactionModeSubscriber_;
    void interactionModeCallback(const CORC::InteractionMode &msg);

    // Subscriber and callback func emg data
    // it receive messages in composite by array of multiple (variable lenght) trignoEMG messages
    // each trignoEMG message brings:  start_time [float64], emg_pos [string], emg_id [int16], emg [float64[]]
    // emg is an array the emg values containing the last N (TODO) emg reading (collected at 2000Hz)
    ros::Subscriber trignoEMGSubscriber_; // trigno EMG subscriber
    void trignoEMGCallback(const trigno_msgs::trignoMultiEMG &msg);

    // Publisher and message for joint state publication
    ros::Publisher jointStatePublisher_;
    sensor_msgs::JointState jointStateMsg_;

    // Publisher and message for interaction wrench publication
    ros::Publisher interactionWrenchPublisher_;
    geometry_msgs::WrenchStamped interactionWrenchMsg_;

    // Publisher and message for arduino scaled force publication
    ros::Publisher interactionScaledPublisher_;
    geometry_msgs::Point32 interactionScaledMsg_;

    // Publisher and message for simple scaled joint messages
    ros::Publisher jointScaledPublisher_;
    CORC::JointScaled32 jointScaledMsg_;

    // Publisher and message for EMG data
    ros::Publisher trignoEMGPublisher_;
    trigno_msgs::trignoMultiEMG trignoMultiEMGMsg_;

    RobotM1 *robot_;
    TrignoMultiEMG *trignoMultiEMG_;
    TrignoMultiIMU *trignoMultiIMU_;

    ros::ServiceServer calibrateForceSensorsService_;
    bool calibrateForceSensorsCallback(std_srvs::Trigger::Request& req,
                                       std_srvs::Trigger::Response& res);

    ros::ServiceServer setMVCService_;
    bool setMVCCallback(CORC::SetMVC::Request& req,
                        CORC::SetMVC::Response& res);

    // Parameters for robot name and muscle count
    RobotParameters m1Params;
    int muscleCount_;

    // Conversion factors between degrees and radians
    double d2r, r2d;

    // Time that passed to update function. Equals to log time.
    double time_;
};

#endif  //SRC_MultiM1MachineROS_H

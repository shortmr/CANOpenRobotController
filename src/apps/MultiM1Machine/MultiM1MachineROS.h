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

class MultiM1MachineROS {
public:
    MultiM1MachineROS(RobotM1 *robot);
    ~MultiM1MachineROS();

    void update(void);
    void publishJointStates(void);
    void publishInteractionForces(void);
    void publishJointScaled(void);
    void initialize();
    void setNodeHandle(ros::NodeHandle& nodeHandle);

    ros::NodeHandle* nodeHandle_;

    Eigen::VectorXd jointPositionCommand_, jointVelocityCommand_, jointTorqueCommand_;
    Eigen::VectorXd interactionTorqueCommand_;
    Eigen::VectorXd prbsPositionCommand_;
    Eigen::VectorXd emgData_;

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

    // Subscriber and callback func emg data
    ros::Subscriber emgDataSubscriber_;
    void emgDataCallback(const std_msgs::Float64MultiArray &msg);

    // Subscriber and callback func emg data
    ros::Subscriber interactionModeSubscriber_;
    void interactionModeCallback(const CORC::InteractionMode &msg);

    // Publisher and message for joint state publication
    ros::Publisher jointStatePublisher_;
    sensor_msgs::JointState jointStateMsg_;

    // Publisher and message for interaction wrench publication
    ros::Publisher interactionWrenchPublisher_;
    geometry_msgs::WrenchStamped interactionWrenchMsg_;

    // Publisher and message for simple scaled joint messages
    ros::Publisher jointScaledPublisher_;
    CORC::JointScaled32 jointScaledMsg_;

    RobotM1 *robot_;

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
};

#endif  //SRC_MultiM1MachineROS_H

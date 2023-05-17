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

class MultiM1MachineROS {
public:
    MultiM1MachineROS(RobotM1 *robot);
    ~MultiM1MachineROS();

    void update(void);
    void publishJointStates(void);
    void publishInteractionForces(void);
    void publishInteractionScaled(void);
    void initialize();
    void setNodeHandle(ros::NodeHandle& nodeHandle);

    ros::NodeHandle* nodeHandle_;

    Eigen::VectorXd jointPositionCommand_, jointVelocityCommand_, jointTorqueCommand_;
    Eigen::VectorXd interactionTorqueCommand_;
    Eigen::VectorXd qFixed_;

private:
    // Subscriber and callback func for joint command subscription
    ros::Subscriber jointCommandSubscriber_;
    void jointCommandCallback(const sensor_msgs::JointState &msg);

    // Subscriber and callback func for interaction torque subscription
    ros::Subscriber interactionTorqueCommandSubscriber_;
    void interactionTorqueCommandCallback(const std_msgs::Float64MultiArray &msg);

    // Publisher and message for joint state publication
    ros::Publisher jointStatePublisher_;
    sensor_msgs::JointState jointStateMsg_;

    // Publisher and message for interaction wrench publication
    ros::Publisher interactionWrenchPublisher_;
    geometry_msgs::WrenchStamped interactionWrenchMsg_;

    // Publisher and message for arduino scaled force publication
    ros::Publisher interactionScaledPublisher_;
    geometry_msgs::Point32 interactionScaledMsg_;

    RobotM1 *robot_;

    ros::ServiceServer calibrateForceSensorsService_;
    bool calibrateForceSensorsCallback(std_srvs::Trigger::Request& req,
                                       std_srvs::Trigger::Response& res);
    ros::ServiceServer setFixedAngleService_;
    bool setFixedAngleCallback(std_srvs::Trigger::Request& req,
                               std_srvs::Trigger::Response& res);

    ros::ServiceServer setTorqueOffsetService_;
    bool setTorqueOffsetCallback(std_srvs::Trigger::Request& req,
                               std_srvs::Trigger::Response& res);
};

#endif  //SRC_MultiM1MachineROS_H
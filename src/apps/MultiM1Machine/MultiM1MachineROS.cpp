#include "MultiM1MachineROS.h"

MultiM1MachineROS::MultiM1MachineROS(RobotM1 *robot) {
    robot_ = robot;
}

MultiM1MachineROS::~MultiM1MachineROS() {
    ros::shutdown();
}

void MultiM1MachineROS::initialize() {
    spdlog::debug("MultiM1MachineROS::init()");

    jointCommandSubscriber_ = nodeHandle_->subscribe("joint_commands", 1, &MultiM1MachineROS::jointCommandCallback, this);
    interactionTorqueCommandSubscriber_ = nodeHandle_->subscribe("interaction_effort_commands", 1, &MultiM1MachineROS::interactionTorqueCommandCallback, this);
    jointStatePublisher_ = nodeHandle_->advertise<sensor_msgs::JointState>("joint_states", 10);
    interactionWrenchPublisher_ = nodeHandle_->advertise<geometry_msgs::WrenchStamped>("interaction_wrench", 10);
    interactionScaledPublisher_ = nodeHandle_->advertise<geometry_msgs::Point32>("interaction_mvc", 10);
    jointScaledPublisher_ = nodeHandle_->advertise<CORC::JointScaled32>("joint_scaled", 10);

    jointPositionCommand_ = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    jointVelocityCommand_ = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    jointTorqueCommand_ = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    interactionTorqueCommand_ = Eigen::VectorXd(M1_NUM_INTERACTION);

    calibrateForceSensorsService_ = nodeHandle_->advertiseService("calibrate_force_sensors", &MultiM1MachineROS::calibrateForceSensorsCallback, this);
}

void MultiM1MachineROS::update() {
    publishJointStates();
    publishInteractionForces();
    publishJointScaled(); // publishInteractionScaled();
}

void MultiM1MachineROS::publishJointStates() {
    Eigen::VectorXd jointPositions = robot_->getPosition();
    Eigen::VectorXd jointVelocities = robot_->getVelocity();
    Eigen::VectorXd jointTorques = robot_->getTorque();

    jointStateMsg_.header.stamp = ros::Time::now()-ros::Duration(5);
    jointStateMsg_.name.resize(M1_NUM_JOINTS);
    jointStateMsg_.position.resize(M1_NUM_JOINTS);
    jointStateMsg_.velocity.resize(M1_NUM_JOINTS);
    jointStateMsg_.effort.resize(M1_NUM_JOINTS);
    jointStateMsg_.name[0] = "M1_joint";
    jointStateMsg_.position[0] = jointPositions[0];
    jointStateMsg_.velocity[0] = jointVelocities[0];
    jointStateMsg_.effort[0] = jointTorques[0]; /// BE CAREFUL CHANGED FROM JOINT TORQUE TO DESIRED INTERACTION TORQUE FOR SINGLE ROBOT FORCE CONTROL TEST

    jointStatePublisher_.publish(jointStateMsg_);
}

void MultiM1MachineROS::publishInteractionForces() {
    Eigen::VectorXd interactionTorque = robot_->getJointTor_s(); // with weight compensation
    ros::Time time = ros::Time::now();

    interactionWrenchMsg_.header.stamp = time;
    interactionWrenchMsg_.header.frame_id = "interaction_torque_sensor";
    interactionWrenchMsg_.wrench.torque.y = robot_->tau_spring[0];
    interactionWrenchMsg_.wrench.torque.z = -interactionTorque[0];
    interactionWrenchMsg_.wrench.torque.x = interactionTorque[0];

    interactionWrenchPublisher_.publish(interactionWrenchMsg_);
}

void MultiM1MachineROS::publishInteractionScaled() {
    Eigen::VectorXd interactionTorqueFiltered = robot_->getJointTor_s_filt(); // filtered with weight compensation
    double torqueScaled;

    if (robot_->stim_calib_) {
        torqueScaled = 1;
    } else {
        if ((interactionTorqueFiltered[0]-robot_->tau_offset_) > 0) {
            torqueScaled = (interactionTorqueFiltered[0]-robot_->tau_offset_)/(robot_->tau_df_);
        } else {
            torqueScaled = (interactionTorqueFiltered[0]-robot_->tau_offset_)/(robot_->tau_pf_);
        }
    }

    interactionScaledMsg_.x = torqueScaled;
    interactionScaledMsg_.y = robot_->stim_df_; // stimulation amplitude channel 1
    interactionScaledMsg_.z = robot_->stim_pf_; // stimulation amplitude channel 2
    interactionScaledPublisher_.publish(interactionScaledMsg_);
}

void MultiM1MachineROS::publishJointScaled() {
    Eigen::VectorXd interactionTorqueFiltered = robot_->getJointTor_s_filt(); // filtered with weight compensation
    Eigen::VectorXd jointAngle = robot_->getJointPos();
    double torqueScaled;
    double angleScaled;
    double tau_df = robot_->tau_df_;
    double tau_pf = robot_->tau_pf_;
    double q_df = robot_->q_df_;
    double q_pf = robot_->q_pf_;
    double tau_offset = robot_->tau_offset_;

    // scale torque
    if ((interactionTorqueFiltered[0]-tau_offset) > 0) {
        torqueScaled = (interactionTorqueFiltered[0]-tau_offset)/tau_df;
    } else {
        torqueScaled = (interactionTorqueFiltered[0]-tau_offset)/tau_pf;
    }

    // scale angle
    if (q_df==0 && q_pf==90) {
        angleScaled = jointAngle[0]; // unscaled
    } else {
        angleScaled = 2*(jointAngle[0] - 0.5*(q_df + q_pf))/(q_df - q_pf);
    }

    jointScaledMsg_.tau_s = torqueScaled; // scaled torque (fraction of MVC different for DF and PF; -1 to 1)
    jointScaledMsg_.q = angleScaled; // scaled angular position (fraction of ROM; -1 to 1)
    jointScaledMsg_.tau_df = tau_df; // maximum torque in dorsiflexion (Nm)
    jointScaledMsg_.tau_pf = tau_pf; // maximum torque in plantarflexion (Nm)
    jointScaledMsg_.q_df = q_df; // maximum angle in dorsiflexion (deg)
    jointScaledMsg_.q_pf = q_pf; // maximum angle in plantarflexion (deg)
    jointScaledPublisher_.publish(jointScaledMsg_);
}

void MultiM1MachineROS::setNodeHandle(ros::NodeHandle &nodeHandle) {
    nodeHandle_ = &nodeHandle;
}

void MultiM1MachineROS::jointCommandCallback(const sensor_msgs::JointState &msg) {

    for(int i=0; i<M1_NUM_JOINTS; i++){
        jointPositionCommand_[i] = msg.position[i];
        jointVelocityCommand_[i] = msg.velocity[i];
        jointTorqueCommand_[i] = msg.effort[i];
    }
}

void MultiM1MachineROS::interactionTorqueCommandCallback(const std_msgs::Float64MultiArray &msg) {

    for(int i=0; i<M1_NUM_JOINTS; i++){
        interactionTorqueCommand_[i] = msg.data[i];
    }
}

bool MultiM1MachineROS::calibrateForceSensorsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    // only use when shaft is not bearing load (pedal)
    res.success = robot_->calibrateForceSensors();
    return true;
}
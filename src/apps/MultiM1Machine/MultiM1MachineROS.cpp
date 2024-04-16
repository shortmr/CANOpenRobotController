#include "MultiM1MachineROS.h"

MultiM1MachineROS::MultiM1MachineROS(RobotM1 *robot) {
    robot_ = robot;
}

MultiM1MachineROS::~MultiM1MachineROS() {
    ros::shutdown();
}

void MultiM1MachineROS::initialize() {
    spdlog::debug("MultiM1MachineROS::init()");

    // Conversion factors between degrees and radians
    d2r = M_PI / 180.;
    r2d = 180. / M_PI;

    jointCommandSubscriber_ = nodeHandle_->subscribe("joint_commands", 1, &MultiM1MachineROS::jointCommandCallback, this);
    interactionTorqueCommandSubscriber_ = nodeHandle_->subscribe("interaction_effort_commands", 1, &MultiM1MachineROS::interactionTorqueCommandCallback, this);
    prbsCommandSubscriber_ = nodeHandle_->subscribe("prbs_commands", 1, &MultiM1MachineROS::prbsCommandCallback, this);
    emgDataSubscriber_ = nodeHandle_->subscribe("emg_data", 1, &MultiM1MachineROS::emgDataCallback, this);
    interactionModeSubscriber_ = nodeHandle_->subscribe("interaction_mode", 1, &MultiM1MachineROS::interactionModeCallback, this);
    jointStatePublisher_ = nodeHandle_->advertise<sensor_msgs::JointState>("joint_states", 10);
    interactionWrenchPublisher_ = nodeHandle_->advertise<geometry_msgs::WrenchStamped>("interaction_wrench", 10);
    interactionScaledPublisher_ = nodeHandle_->advertise<geometry_msgs::Point32>("interaction_mvc", 10);
    jointScaledPublisher_ = nodeHandle_->advertise<CORC::JointScaled32>("joint_scaled", 10);

    jointPositionCommand_ = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    jointVelocityCommand_ = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    jointTorqueCommand_ = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    interactionTorqueCommand_ = Eigen::VectorXd(M1_NUM_INTERACTION);
    prbsPositionCommand_ = Eigen::VectorXd::Zero(M1_NUM_JOINTS);

    jointPositionScaled_ = Eigen::VectorXd::Zero(1);
    jointTorqueScaled_ = Eigen::VectorXd::Zero(1);

    m1Params = robot_->sendRobotParams();
    muscleCount_ = m1Params.muscle_count[0];
    emgData_ = Eigen::VectorXd::Zero(muscleCount_);

    calibrateForceSensorsService_ = nodeHandle_->advertiseService("calibrate_force_sensors", &MultiM1MachineROS::calibrateForceSensorsCallback, this);
}

void MultiM1MachineROS::update() {
    publishJointStates();
    publishInteractionForces();
    publishJointScaled();
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
    jointStateMsg_.position[0] = jointPositions[0]-robot_->getPositionOffset(); // remove bias (center of ROM from position)
    jointStateMsg_.velocity[0] = jointVelocities[0];
    jointStateMsg_.effort[0] = jointTorques[0]; /// BE CAREFUL CHANGED FROM JOINT TORQUE TO DESIRED INTERACTION TORQUE FOR SINGLE ROBOT FORCE CONTROL TEST

    jointStatePublisher_.publish(jointStateMsg_);
}

void MultiM1MachineROS::publishInteractionForces() {
    Eigen::VectorXd interactionTorque = robot_->getJointTor_s(); // with weight compensation
    Eigen::VectorXd interactionTorqueFiltered = robot_->getJointTor_s_filt(); // filtered with weight compensation
    double torqueOffset = robot_->getTorqueOffset(); // torque offset in Nm
    ros::Time time = ros::Time::now();

    interactionWrenchMsg_.header.stamp = time;
    interactionWrenchMsg_.header.frame_id = "interaction_torque_sensor";
    interactionWrenchMsg_.wrench.torque.y = robot_->tau_spring[0];
    interactionWrenchMsg_.wrench.torque.z = interactionTorqueFiltered[0]-torqueOffset; // -interactionTorqueFiltered[0]
    interactionWrenchMsg_.wrench.torque.x = interactionTorque[0];

    interactionWrenchPublisher_.publish(interactionWrenchMsg_);
}

void MultiM1MachineROS::publishInteractionScaled() {
    Eigen::VectorXd interactionTorqueFiltered = robot_->getJointTor_s_filt(); // filtered with weight compensation
    Eigen::VectorXd torqueLimits = robot_->getTorqueLimits(); // torque limits in Nm
    double torqueOffset = robot_->getTorqueOffset(); // torque offset in Nm
    double torqueScaled;

    if (robot_->stim_calib_) {
        torqueScaled = 1;
    } else {
        // scale torque
        if ((interactionTorqueFiltered[0]-torqueOffset) > 0) {
            torqueScaled = (interactionTorqueFiltered[0]-torqueOffset)/torqueLimits[0]; // DF
        } else {
            torqueScaled = (interactionTorqueFiltered[0]-torqueOffset)/torqueLimits[1]; // PF
        }
    }

    interactionScaledMsg_.x = torqueScaled;
    interactionScaledMsg_.y = robot_->stim_df_; // stimulation amplitude channel 1
    interactionScaledMsg_.z = robot_->stim_pf_; // stimulation amplitude channel 2
    interactionScaledPublisher_.publish(interactionScaledMsg_);
}

void MultiM1MachineROS::publishJointScaled() {
    Eigen::VectorXd interactionTorqueFiltered = robot_->getJointTor_s_filt(); // filtered with weight compensation
    Eigen::VectorXd jointPositions = robot_->getPosition(); // angular position in radians
    Eigen::VectorXd torqueLimits = robot_->getTorqueLimits(); // torque limits in Nm
    Eigen::VectorXd positionLimits = robot_->getPositionLimits(); // angular position limits in radians
    Eigen::VectorXd jointVelocities = robot_->getVelocity(); // angular position in radians
    double torqueOffset = robot_->getTorqueOffset(); // torque offset in Nm
    double q_bias = robot_->t_bias_; // angle between lower joint limit and zero_calibration angle

    // scale torque
    if ((interactionTorqueFiltered[0]-torqueOffset) > 0) {
        jointTorqueScaled_[0] = (interactionTorqueFiltered[0]-torqueOffset)/torqueLimits[0]; // DF
    } else {
        jointTorqueScaled_[0] = (interactionTorqueFiltered[0]-torqueOffset)/torqueLimits[1]; // PF
    }

    // scale angle
    if (positionLimits[0]==positionLimits[1]) {
        jointPositionScaled_[0] = jointPositions[0]; // unscaled
    } else {
        jointPositionScaled_[0] = 2*(jointPositions[0] - 0.5*(positionLimits[0] + positionLimits[1]))/(positionLimits[0] - positionLimits[1]);
    }

    jointScaledMsg_.tau_s = jointTorqueScaled_[0]; // scaled torque (fraction of MVC different for DF and PF; -1 to 1)
    jointScaledMsg_.q = jointPositionScaled_[0]; // scaled angular position (fraction of ROM; -1 to 1)
    jointScaledMsg_.tau_df = torqueLimits[0]; // maximum torque in dorsiflexion (Nm)
    jointScaledMsg_.tau_pf = torqueLimits[1]; // maximum torque in plantarflexion (Nm)
    jointScaledMsg_.q_df = positionLimits[0]; // maximum angle in dorsiflexion (rad)
    jointScaledMsg_.q_pf = positionLimits[1]; // maximum angle in plantarflexion (rad)
    jointScaledMsg_.q_bias = q_bias; // bias angle (rad)
    jointScaledMsg_.dq = jointVelocities[0]; // velocity (rad/s)
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

void MultiM1MachineROS::prbsCommandCallback(const geometry_msgs::Vector3 &msg) {

    for(int i=0; i<M1_NUM_JOINTS; i++){
        prbsPositionCommand_[i] = msg.y;
    }
}

void MultiM1MachineROS::emgDataCallback(const std_msgs::Float64MultiArray &msg) {
    for(int i=0; i<muscleCount_; i++){
        emgData_[i] = msg.data[i];
    }
}

void MultiM1MachineROS::interactionModeCallback(const CORC::InteractionMode &msg) {
    interactionMode_ = msg.mode;
    referenceLimb_ = msg.reference;
}

bool MultiM1MachineROS::calibrateForceSensorsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    // only use when shaft is not bearing load (pedal)
    res.success = robot_->calibrateForceSensors();
    return true;
}
#include "MultiControllerState.h"

double timeval_to_sec_t(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

void MultiControllerState::entry(void) {

    spdlog::info("Multi Controller State is entered.");

    //Timing
    clock_gettime(CLOCK_MONOTONIC, &initTime);
    lastTime = timeval_to_sec_t(&initTime);

    // Set up dynamic parameter server
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&MultiControllerState::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

    // Initialize static parameters (if configFlag is not set, bypass dynamic reconfigure for parameters)
    m1Params = robot_->sendRobotParams();
    v_bias = -1 * m1Params.t_bias[0] * 180. / M_PI;
    tick_max_ = m1Params.tick_max[0];
    spk_ = m1Params.spk[0];
    cut_off_ = m1Params.lowpass_cutoff_freq[0];
    robot_->setVelThresh(m1Params.vel_thresh[0]);
    robot_->setTorqueThresh(m1Params.tau_thresh[0]);
    robot_->setMotorTorqueCutOff(m1Params.motor_torque_cutoff_freq[0]);
    if (!m1Params.configFlag) {
        // Update feedforward and PID gains from yaml parameter file
        ffRatio_ = m1Params.ff_ratio[0];
        kp_ = m1Params.kp[0];
        ki_ = m1Params.ki[0];
        kd_ = m1Params.kd[0];
    }

    robot_->initTorqueControl();
    robot_->tau_spring[0] = 0;   // for ROS publish only

    // Transparency vectors
    q = Eigen::VectorXd::Zero(1);
    dq = Eigen::VectorXd::Zero(1);
    tau = Eigen::VectorXd::Zero(1);
    tau_s = Eigen::VectorXd::Zero(1);
    tau_cmd = Eigen::VectorXd::Zero(1);

    // Initialize parameters
    control_freq = 400.0;
    error = 0;
    delta_error = 0;
    integral_error = 0;
    tick_count = 0;
    cali_tau_thresh = -13;
    cali_vel_thresh = 2;

    controller_mode_ = -1;

    // System identification
    cycle = 0;
    id_mode = 1; // sine = 1; ramp = 2
    counter = 0;
    max_cycle = 4;
    freq = 0;
    mag = 0;
    s_mag = 0;
    s_mag_prev = 0;
    step = 0;
    max_tau = 4;
    dir = true;

    digitalInValue_ = 0;
    digitalOutValue_ = 0;
    robot_->setDigitalOut(digitalOutValue_);
    robot_->setControlFreq(control_freq);
}
void MultiControllerState::during(void) {

    //Compute some basic time values
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    double now = timeval_to_sec_t(&ts);
    elapsedTime = (now-timeval_to_sec_t(&initTime));
    dt = now - lastTime;
    lastTime = now;

    tick_count = tick_count + 1;
    if (controller_mode_ == 0) {  // homing (only needs to be performed once after M1 is turned ON)
        if (cali_stage == 1) {
            // set calibration velocity
            JointVec dq_t;
            dq_t(0) = cali_velocity; // calibration velocity
            if (robot_->setJointVel(dq_t) != SUCCESS) {
                std::cout << "Error: " << std::endl;
            }

            // monitor velocity and interaction torque
            dq= robot_->getJointVel();
            tau = robot_->getJointTor();
            if ((dq(0) <= cali_vel_thresh) & (tau(0) <= cali_tau_thresh)) {
                cali_velocity = 0;
                robot_->applyCalibration();
                robot_->initPositionControl();
                cali_stage = 2;
            } else {
                robot_->printJointStatus();
            }
        } else if (cali_stage == 2) {
            // set position control to vertical
            JointVec q_t;
            q_t(0) = v_bias;
            if(robot_->setJointPos(q_t) != SUCCESS){
                std::cout << "Error: " << std::endl;
            }

            // monitor position
            q = robot_->getJointPos();
            if (abs(q(0)-v_bias)<0.001){
                std::cout << "Calibration done!" << std::endl;
                cali_stage = 3;
            }
            else {
                robot_->printJointStatus();
            }
        }
    }
    else if (controller_mode_ == 1) {  // zero torque mode
        robot_->setJointTor(Eigen::VectorXd::Zero(M1_NUM_JOINTS));
    }
    else if (controller_mode_ == 2) { // virtual spring - torque mode
        tau = robot_->getJointTor();

        // filter position
        q = robot_->getJointPos();
        q_raw = q(0);
        alpha_q = (2*M_PI*dt*cut_off_)/(2*M_PI*dt*cut_off_+1);
        q_filtered = robot_->filter_q(alpha_q);

        // filter velocity
        dq = robot_->getJointVel();
        dq_raw = dq(0);
        alpha_dq = (2*M_PI*dt*cut_off_)/(2*M_PI*dt*cut_off_+1);
        dq_filtered = robot_->filter_dq(alpha_dq);

        // filter interaction torque
        tau_s = robot_->getJointTor_s();
        tau_raw = tau_s(0);
        alpha_tau_s = (2*M_PI*dt*cut_off_)/(2*M_PI*dt*cut_off_+1);
        tau_filtered = robot_->filter_tau_s(alpha_tau_s);

        // get interaction torque from virtual spring
        spring_tor = -multiM1MachineRos_->interactionTorqueCommand_(0);
        robot_->tau_spring[0] = spring_tor; // for ROS publish only

        // apply PID for feedback control
        error = tau_filtered + spring_tor;  // interaction torque error (desired interaction torque is spring_tor)

        delta_error = (error-torque_error_last_time_step)*control_freq;  // derivative of interaction torque error
        integral_error = integral_error + error/control_freq; // integral of interaction torque error
        tau_cmd(0) = error*kp_ + delta_error*kd_ + integral_error*ki_;
        torque_error_last_time_step = error;
        robot_->setJointTor_comp(tau_cmd, ffRatio_);

        // reset integral_error every n seconds (tick_max_)
        if(tick_count >= control_freq*tick_max_){
            integral_error = 0;
            tick_count = 0;
        }
    }
}

void MultiControllerState::exit(void) {
    robot_->initTorqueControl();
    robot_->setJointTor(Eigen::VectorXd::Zero(M1_NUM_JOINTS));

}

void MultiControllerState::dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level) {
    // Update PID and feedforward gains from RQT GUI
    if (m1Params.configFlag) {
        ffRatio_ = config.ff_ratio;
        kp_ = config.kp;
        kd_ = config.kd;
        ki_ = config.ki;
    } else {
        std::cout << "Dynamic reconfigure parameter setting is disabled (set configFlag to true to enable)" << std::endl;
    }

    // Change control mode on RQT GUI change
    if(controller_mode_!=config.controller_mode)
    {
        controller_mode_ = config.controller_mode;
        if(controller_mode_ == 0) {
            robot_->initVelocityControl();
            cali_stage = 1;
            cali_velocity = -30;
        }

        if (controller_mode_ == 1) robot_->initTorqueControl();
        if (controller_mode_ == 2) robot_->initTorqueControl();
    }

    return;
}



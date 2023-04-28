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
    if (!m1Params.configFlag) {
        // Update PID and feedforward gains from yaml parameter file
        kp_ = m1Params.kp[0];
        ki_ = m1Params.ki[0];
        kd_ = m1Params.kd[0];
        cut_off_ = m1Params.lowpass_cutoff_freq[0];
        ffRatio_ = m1Params.ff_ratio[0];
        spk_ = m1Params.spk[0];
        tick_max_ = m1Params.tick_max[0];
    }

    robot_->initTorqueControl();
    robot_->tau_spring[0] = 0;   // for ROS publish only

    //robot_->applyCalibration();
    //robot_->calibrateForceSensors();

    // Transparency vectors
    q = Eigen::VectorXd::Zero(1);
    dq = Eigen::VectorXd::Zero(1);
    tau = Eigen::VectorXd::Zero(1);
    tau_s = Eigen::VectorXd::Zero(1);
    tau_cmd = Eigen::VectorXd::Zero(1);

    // Initialize parameters
    control_freq = 400.0; //400
    error = 0;
    delta_error = 0;
    integral_error = 0;
    tick_count = 0;
    cali_tau_thresh = -13;
    cali_vel_thresh = 2;

    controller_mode_ = -1;
//    cut_off_ = 6.0;

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
    else if (controller_mode_ == 2) { // follow position commands
        robot_->setJointPos(multiM1MachineRos_->jointPositionCommand_);
    }
    else if (controller_mode_ == 3) { // follow torque commands
        robot_->setJointTor(multiM1MachineRos_->jointTorqueCommand_);
    }
    else if (controller_mode_ == 4 || controller_mode_ == 5) { // virtual spring or transparency - torque mode
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
        if (controller_mode_ == 4) {
            error = tau_filtered + spring_tor;  // interaction torque error (desired interaction torque is spring_tor)
        }
        else if (controller_mode_ == 5) {
            error = tau_filtered;  // interaction torque error (desired interaction torque is 0)
        }
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
    else if (controller_mode_ == 6) {  // fixed neutral - zero velocity mode
        if (fixed_stage == 1) {
            // set calibration velocity
            JointVec dq_t;
            dq_t(0) = cali_velocity;
            if (robot_->setJointVel(dq_t) != SUCCESS) {
                std::cout << "Error: " << std::endl;
            }

            // monitor velocity and joint torque
            dq = robot_->getJointVel();
            tau = robot_->getJointTor();
            if ((dq(0) <= cali_vel_thresh) & (tau(0) <= cali_tau_thresh)) {
                cali_velocity = 0;
                robot_->initPositionControl();
                fixed_stage = 2;
            } else {
                robot_->printJointStatus();
            }
        }
        if (fixed_stage == 2) {
            // set fixed neutral position
            JointVec q_t;
            q_t(0) = fixed_q;
            if(robot_->setJointPos(q_t) != SUCCESS){
                std::cout << "Error: " << std::endl;
            }

            // monitor position
            q = robot_->getJointPos();
            if (abs(q(0)-fixed_q)<0.001){
                robot_->initVelocityControl();
                std::cout << "Holding neutral position with zero velocity" << std::endl;
                fixed_stage = 3;
            }
            else {
                robot_->printJointStatus();
            }
        } else if (fixed_stage == 3) {
            // apply zero velocity mode
            robot_->setJointVel(Eigen::VectorXd::Zero(M1_NUM_JOINTS));
        }
    }
    else if (controller_mode_ == 7) {  // fixed angle - zero velocity mode (slider angle selection)
        if (fixed_stage == 2) {
            // set fixed neutral position
            JointVec q_t;
            q_t(0) = slider_q;
            if(robot_->setJointPos(q_t) != SUCCESS){
                std::cout << "Error: " << std::endl;
            }

            // monitor position
            q = robot_->getJointPos();
            if (abs(q(0)-slider_q)<0.001){
                robot_->initVelocityControl();
                std::cout << "Holding user position with zero velocity" << std::endl;
                fixed_stage = 3;
            }
            else {
                robot_->printJointStatus();
            }
        } else if (fixed_stage == 3) {
            // apply zero velocity mode
            robot_->setJointVel(Eigen::VectorXd::Zero(M1_NUM_JOINTS));
        }

//        //filter interaction torque for arduino publishing
//        alpha_tau_s = (2*M_PI*dt*cut_off_)/(2*M_PI*dt*cut_off_+1);
//        tau_filtered = robot_->filter_tau_s(alpha_tau_s);
    }
    else if(controller_mode_ == 8) {  // system identification - torque mode
        counter = counter + 1;
        if(counter%100==1) {
            robot_->printJointStatus();
        }

        // change frequency/magnitude/step amount after max_cycle
        if(cycle >= max_cycle) {
            cycle = 0;
            start = true;
            counter = 0;

            if (id_mode == 1) {
                mag = mag + 0.2; // sine wave magnitude (torque)
                freq = freq + 0.1; // sine wave frequency (torque)
                if (mag > 5.0) {
                    freq = 0.1;
                    mag = 3;
                    id_mode = 2;
                    std::cout << "switch to ramp" << std::endl;
                }
            } else if (id_mode == 2) {
                step = step + 0.2; // slope of ramp (torque)
                if (step > 1.00) {
                    step = 0.1;
                    max_tau = 4;
                    id_mode = 1;
                    std::cout << "switch to sine" << std::endl;
                }
            }
        }

        q = robot_->getJointPos();
        if (id_mode == 1) {
            // set sinusoidal torque input
            if (cycle == 0 && start) {
                start = false;
                std::cout << std::setprecision(2) << "torque magnitude: " << mag << "; frequency: " << freq
                          << "; Cycle: " << cycle << std::endl;
            }
            s_mag = sin(2 * M_PI * freq * counter / control_freq);
            tau_cmd(0) = mag * s_mag + 0.8; //mag * s_mag + 0.8;
            if(s_mag_prev < 0 && s_mag > 0)
            {
                cycle = cycle + 1;
                if (cycle < max_cycle) {
                    std::cout << std::setprecision(2) << "torque magnitude: " << mag << "; frequency: " << freq
                              << "; Cycle: " << cycle << std::endl;
                }
            }
            s_mag_prev = s_mag;
        } else if (id_mode == 2) {
            // set ramp torque input
            if (cycle == 0 && start) {
                start = false;
                std::cout << std::setprecision(2) << "step magnitude: " << step << "; Cycle: " << cycle
                          << std::endl;
            }
            // change torque direction while monitoring angle between 10 and 80 degrees
            if (dir) {
                if (q(0) < 80) {
                    tau_cmd(0) = tau_cmd(0) + step;
                } else {
                    dir = false;
                }
            } else {
                if (q(0) > 10) {
                    tau_cmd(0) = tau_cmd(0) - step;
                } else {
                    dir = true;
                    cycle = cycle + 1;
                    if (cycle < max_cycle) {
                        std::cout << std::setprecision(2) << "step magnitude: " << step << "; Cycle: " << cycle
                                  << std::endl;
                    }
                }
            }
            // set upper and lower limits of ramp torque command
            if (tau_cmd(0) > max_tau) {
                tau_cmd(0) = max_tau;
            } else if (tau_cmd(0) < -1 * max_tau) {
                tau_cmd(0) = -1 * max_tau;
            }
        }
        // set joint torque command
        if (robot_->setJointTor(tau_cmd) != SUCCESS) {
            std::cout << "Error: " << std::endl;
        }
    }
    else if (controller_mode_ == 11) { // Send high for external trigger with NI DAQ

        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        if(time > 2.0){
            if (digitalOutValue_ == 1) {
                digitalOutValue_ = 0;
                robot_->setDigitalOut(digitalOutValue_);
            }
        }
        else if(time > 1.0){
            if (digitalOutValue_ == 0) {
                digitalOutValue_ = 1;
                robot_->setDigitalOut(digitalOutValue_);
            }
        }
    }
    // Read setDigitalOut signal
    // digitalInValue_ = robot_->getDigitalIn();
}

void MultiControllerState::exit(void) {
    robot_->initTorqueControl();
    robot_->setJointTor(Eigen::VectorXd::Zero(M1_NUM_JOINTS));

}

void MultiControllerState::dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level) {
    // Update PID and feedforward gains from RQT GUI
    if (m1Params.configFlag) {
        kp_ = config.kp;
        kd_ = config.kd;
        ki_ = config.ki;
        robot_->setVelThresh(config.vel_thresh);
        robot_->setTorqueThresh(config.tau_thresh);
        robot_->setMotorTorqueCutOff(config.motor_torque_cutoff_freq);
        robot_->setStaticFriction(config.c0);
        robot_->setDynamicFriction(config.c1);

        cut_off_ = config.lowpass_cutoff_freq;
        ffRatio_ = config.ff_ratio;
        spk_ = config.spk;

        if(tick_max_ != config.tick_max)
        {
            tick_max_ = config.tick_max;
            tick_count = 0;
        }
    } else {
        std::cout << "Dynamic reconfigure parameter setting is disabled (set configFlag to true to enable)" << std::endl;
    }

    slider_angle_ = config.slider_angle + v_bias; //relative to vertical position

    // arduino stimulation parameters
    robot_->setMaxDF(config.max_torque_df);
    robot_->setMaxPF(config.max_torque_pf);
    robot_->setStimDF(config.stim_amp_df);
    robot_->setStimPF(config.stim_amp_pf);
    robot_->setStimCalibrate(config.stim_calibrate);

    // Change control mode on RQT GUI change
    if(controller_mode_!=config.controller_mode)
    {
        controller_mode_ = config.controller_mode;
        if(controller_mode_ == 0) {
            robot_->initVelocityControl();
            cali_stage = 1;
            cali_velocity = -30;
        }

        cycle = 0;
        counter = 0;

        if (controller_mode_ == 1) robot_->initTorqueControl();
        if (controller_mode_ == 2) robot_->initPositionControl();
        if (controller_mode_ == 3) robot_->initTorqueControl();
        if (controller_mode_ == 4) robot_->initTorqueControl();
        if (controller_mode_ == 5) robot_->initTorqueControl();

        if (controller_mode_ == 6) {
            robot_->initVelocityControl();
            fixed_stage = 1;
            fixed_q = 45; // trajectory bias = 45
            cali_velocity = -30;
        }
        if (controller_mode_ == 7) {
            robot_->initPositionControl();
            fixed_stage = 2;
            slider_q = slider_angle_; // get updated slider value (force calibration = 90+v_bias)
        }
        if (controller_mode_ == 8) {
            robot_->initTorqueControl();
            freq = 0.1;
            mag = 3;   // magnitude for sine wave (without compensation = 3, with compensation = 0.6)
            s_mag = 0; // current value of sine wave
            s_mag_prev = 0; // previous value of sine wave
            step = 0.1; // step for ramp
            dir = true;
            start = true;
        }
        if (controller_mode_ == 11) time0 = std::chrono::steady_clock::now();
        if (controller_mode_ == 11) robot_->setDigitalOut(0);
    }

    return;
}



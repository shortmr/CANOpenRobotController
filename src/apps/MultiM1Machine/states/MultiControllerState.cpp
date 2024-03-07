#include "MultiControllerState.h"

double timeval_to_sec_t(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

void MultiControllerState::entry(void) {

    spdlog::info("Multi Controller State is entered.");

    // Conversion factors between degrees and radians
    d2r = M_PI / 180.;
    r2d = 180. / M_PI;

    //Timing
    clock_gettime(CLOCK_MONOTONIC, &initTime);
    lastTime = timeval_to_sec_t(&initTime);

    control_freq = 1/0.003; //400 Hz
    robot_->setControlFreq(control_freq);

    // Set up dynamic parameter server
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&MultiControllerState::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

    // Initialize static parameters (if configFlag is not set, bypass dynamic reconfigure for parameters)
    m1Params = robot_->sendRobotParams();
    v_bias = -1 * m1Params.t_bias[0] * 180. / M_PI;
    rom_center = m1Params.tracking_offset[0];
    rom_df = m1Params.tracking_df[0];
    rom_pf = m1Params.tracking_pf[0];
    mvc_df = m1Params.mvc_df[0];
    mvc_pf = m1Params.mvc_pf[0];
    if (!m1Params.configFlag) {
        // Update PID and feedforward gains from yaml parameter file
        kp_ = m1Params.kp[0];
        ki_ = m1Params.ki[0];
        kd_ = m1Params.kd[0];
        robot_->setMotorTorqueCutOff(m1Params.motor_torque_cutoff_freq[0]);
        robot_->setSensorCutOff(m1Params.sensor_cutoff_freq[0]);
//        ffRatio_ = m1Params.ff_ratio[0];
        fRatio_ = m1Params.friction_ratio[0];
        wRatio_ = m1Params.weight_ratio[0];
        tick_max_ = m1Params.tick_max[0];
        vel_thresh_ = m1Params.vel_thresh[0];
        kp_mod_ = m1Params.kp_mod[0];
    }
    robot_->initTorqueControl();
    robot_->tau_spring[0] = 0;   // for ROS publish only

    //robot_->applyCalibration();
    //robot_->calibrateForceSensors();

    // Interaction torque control vectors
    q = Eigen::VectorXd::Zero(1);
    dq = Eigen::VectorXd::Zero(1);
    tau = Eigen::VectorXd::Zero(1);
    tau_s = Eigen::VectorXd::Zero(1);
    tau_cmd = Eigen::VectorXd::Zero(1);

    // Initialize parameters
    error = 0;
    delta_error = 0;
    integral_error = 0;
    tick_count = 0;
    cali_tau_thresh = -15;
    cali_tau_safety = 21;
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

            // monitor velocity and motor torque
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

            // safety tau
            if (abs(tau(0)) >= cali_tau_safety) {
                robot_->initTorqueControl();
                std::cout << "Calibration safety error!" << std::endl;
                cali_stage = 4;
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
        } else if (cali_stage == 4) {
            // safety tau default
            robot_->setJointTor(Eigen::VectorXd::Zero(M1_NUM_JOINTS));
        }
    }
    else if (controller_mode_ == 1) {  // zero torque mode
        robot_->setJointTor(Eigen::VectorXd::Zero(M1_NUM_JOINTS));
    }
    else if (controller_mode_ == 2) { // follow position commands
        JointVec q_cmd;
        q_cmd = multiM1MachineRos_->jointPositionCommand_;
        if (q_cmd(0) > -0.75 && q_cmd(0) < 0.75) {
            q_cmd(0) = r2d*q_cmd(0) + rom_center;
        }
        else {
            q_cmd(0) = rom_center;
            std::cout << "Position command out of range" << std::endl;
        }
        if(robot_->setJointPos(q_cmd) != SUCCESS){
            std::cout << "Error: " << std::endl;
        }
        //get filtered interaction torque for subject-specific torque measures
        tau_s_filtered = robot_->getJointTor_s_filt();

        if (set_offset_) {
            n_offset += 1;
            mvc_offset = (mvc_offset+tau_s_filtered(0));
        }

        if (set_mvc_) {
            if (tau_s_filtered(0) > mvc_df) {
                mvc_df = tau_s_filtered(0);
            }
            if (tau_s_filtered(0) < mvc_pf) {
                mvc_pf = tau_s_filtered(0);
            }
        }
    }
    else if (controller_mode_ == 3) { // follow torque commands
        robot_->setJointTor(multiM1MachineRos_->jointTorqueCommand_);
    }
    else if (controller_mode_ == 4 || controller_mode_ == 5) { // virtual spring or transparency - torque mode
        tau = robot_->getJointTor();

        // get position and check for ROM update
        q = robot_->getJointPos(); //degrees
        if (set_rom_) {
            if (q(0) > rom_df) {
                rom_df = q(0);
            }
            if (q(0) < rom_pf) {
                rom_pf = q(0);
            }
        }

        // get velocity
        dq = robot_->getJointVel(); //degrees per second

        // get filtered interaction torque
        tau_s_filtered = robot_->getJointTor_s_filt();

        // get desired interaction torque from virtual spring
        spring_tor = multiM1MachineRos_->interactionTorqueCommand_(0);
        robot_->tau_spring[0] = spring_tor; // for ROS publish only

        // apply PID for feedback control
        if (controller_mode_ == 4) {
            error = tau_s_filtered(0) + spring_tor;  // interaction torque error (desired interaction torque is spring_tor)
        }
        else if (controller_mode_ == 5) {
            error = tau_s_filtered(0);  // interaction torque error (desired interaction torque is 0)
        }
        delta_error = (error-torque_error_last_time_step)*control_freq;  // derivative of interaction torque error
        integral_error = integral_error + error/control_freq; // integral of interaction torque error

        double kp;
        // Vary proportional gain based on velocity (if kp_mod_ == 0, constant proportional gain)
        if (abs(dq(0)) < vel_thresh_) {
            kp = kp_*(1+kp_mod_*(1 - abs(dq(0))/vel_thresh_));
        }
        else {
            kp = kp_;
        }

        tau_cmd(0) = error*kp + delta_error*kd_ + integral_error*ki_;
        torque_error_last_time_step = error;
        robot_->setJointTor_comp(tau_cmd, fRatio_,wRatio_);

        // reset integral_error every n seconds (tick_max_)
        if(tick_count >= control_freq*tick_max_){
            integral_error = 0;
            tick_count = 0;
        }
    }
    else if (controller_mode_ == 6) {  // step angle - zero velocity mode
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        if (fixed_stage_ == 2) {
            // set step angle
            JointVec q_t;
            q_t(0) = step_angle_;
            if(robot_->setJointPos(q_t) != SUCCESS){
                std::cout << "Error: " << std::endl;
            }

            // monitor position
            q = robot_->getJointPos();
            if (abs(q(0)-step_angle_)<0.001){
                robot_->initVelocityControl();
                std::cout << "Holding user position with zero velocity" << std::endl;
                fixed_stage_ = 3;
                time0 = std::chrono::steady_clock::now();
            }
            else {
                robot_->printJointStatus();
            }
        } else if (fixed_stage_ == 3) {
            if (time > 4.0 && step_angle_ <= 100.0 && step_angle_ >= 10.0) {
                robot_->initPositionControl();
                fixed_stage_ = 2;
                step_angle_ = step_angle_ + 10.0;
            }
            else {
                robot_->setJointVel(Eigen::VectorXd::Zero(M1_NUM_JOINTS));
            }
        }
    }
    else if (controller_mode_ == 7) {  // center angle (ROM) - zero velocity mode
        if (fixed_stage_ == 2) {
            // set fixed center angle
            JointVec q_t;
            q_t(0) = rom_center;
            if(robot_->setJointPos(q_t) != SUCCESS){
                std::cout << "Error: " << std::endl;
            }

            // monitor position
            q = robot_->getJointPos();
            if (abs(q(0)-rom_center)<0.001){
                robot_->initVelocityControl();
                std::cout << "Holding user position with zero velocity" << std::endl;
                fixed_stage_ = 3;
            }
            else {
                robot_->printJointStatus();
            }
        } else if (fixed_stage_ == 3) {
            // apply zero velocity mode
            robot_->setJointVel(Eigen::VectorXd::Zero(M1_NUM_JOINTS));
        }

        //filter interaction torque for subject-specific torque measures
        tau_s_filtered = robot_->getJointTor_s_filt();

        if (set_offset_) {
            n_offset += 1;
            mvc_offset = (mvc_offset+tau_s_filtered(0));
        }

        if (set_mvc_) {
            if (tau_s_filtered(0) > mvc_df) {
                mvc_df = tau_s_filtered(0);
            }
            if (tau_s_filtered(0) < mvc_pf) {
                mvc_pf = tau_s_filtered(0);
            }
        }
    }
    else if (controller_mode_ == 8) { // passive rom - velocity mode
        JointVec dq_t = multiM1MachineRos_->jointVelocityCommand_;
        // monitor joint angle
        q = robot_->getJointPos();
        if (set_rom_) {
            robot_->printJointStatus();
            if (q(0) > rom_df) {
                rom_df = q(0);
            }
            if (q(0) < rom_pf) {
                rom_pf = q(0);
            }
        }
        else {
            dq_t(0) = 0.0; // set zero velocity
        }

        // safety feature: restrict absolute velocity to under 2 deg/s, between joint limits
        if ((abs(dq_t(0)) > 2.0) || (dq_t(0) < 0.0 && q(0) < 0.0) || (dq_t(0) > 0.0 && q(0) > 115.0)) {
            dq_t(0) = 0.0;
        }

        // set velocity for joint
        if (robot_->setJointVel(dq_t) != SUCCESS) {
            std::cout << "Error " << std::endl;
        }
    }
    else if(controller_mode_ == 10) {  // system identification - torque mode
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
                if (mag > 4.6) {
                    freq = 0.1;
                    mag = 2.6;
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
            tau_cmd(0) = mag * s_mag + 0.2; //mag * s_mag + 0.8;
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
    else if (controller_mode_ == 12) {  // proprioception: monitor velocity commands
        JointVec dq_t = multiM1MachineRos_->jointVelocityCommand_;
        // monitor joint angle (only move within ROM limits)
        q = robot_->getJointPos();
        if (abs(dq_t(0)) > 0.0) {
            robot_->printJointStatus();
            if ((dq_t(0) < 0.0 && q(0) < rom_pf) || (dq_t(0) > 0.0 && q(0) > rom_df)) {
                dq_t(0) = 0.0;
            }
        }
        // set velocity for joint
        if (robot_->setJointVel(dq_t) != SUCCESS) {
            std::cout << "Error " << std::endl;
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
        vel_thresh_ = config.vel_thresh;
        robot_->setVelThresh(config.vel_thresh);
        robot_->setTorqueThresh(config.tau_thresh);
        robot_->setMotorTorqueCutOff(config.motor_torque_cutoff_freq);
        robot_->setSensorCutOff(config.sensor_cutoff_freq);

//        ffRatio_ = config.ff_ratio;
        fRatio_ = config.friction_ratio;
        wRatio_ = config.weight_ratio;
        kp_mod_ = config.kp_mod;
        robot_->setStaticFrictionFlag(kp_mod_);

        // Hysteresis friction
        robot_->setFrictionParams(config.f_s_hys, config.f_d_hys);

        if(tick_max_ != config.tick_max)
        {
            tick_max_ = config.tick_max;
            tick_count = 0;
        }
    } else {
        std::cout << "Dynamic reconfigure parameter setting is disabled (set configFlag to true to enable)" << std::endl;
    }

    // Arduino stimulation parameters
    robot_->setStimDF(config.stim_amp_df);
    robot_->setStimPF(config.stim_amp_pf);
    robot_->setStimCalibrate(config.stim_calibrate);

    // Switch between ROM measurement
    if(set_rom_!=config.set_rom) {
        set_rom_ = config.set_rom;
        if (!set_rom_) {
            // End measurement
            if (rom_df == 0 && rom_pf == 90) {
                std::cout << "ROM measurement error " << std::endl;
            }
            rom_center = 0.5*(rom_df+rom_pf);
            std::cout << std::setprecision(2) << "Maximum DF angle: " << rom_df << std::endl;
            std::cout << std::setprecision(2) << "Maximum PF angle: " << rom_pf << std::endl;
            std::cout << std::setprecision(2) << "Center angle: " << rom_center << std::endl;
            robot_->setMaxAngleDF(rom_df);
            robot_->setMaxAnglePF(rom_pf);
            robot_->setPositionOffset(rom_center);
        } else {
            std::cout << "Begin ROM measurement... " << std::endl;
            rom_df = 0;
            rom_pf = 90;
            rom_center = 45;
        }
    }
    // Switch between MVC measurement
    if(set_mvc_!=config.set_mvc) {
        set_mvc_ = config.set_mvc;
        if (!set_mvc_) {
            // End measurement
            std::cout << std::setprecision(2) << "Maximum DF torque: " << mvc_df << std::endl;
            std::cout << std::setprecision(2) << "Maximum PF torque: " << mvc_pf << std::endl;
            robot_->setMaxTorqueDF(mvc_df);
            robot_->setMaxTorquePF(abs(mvc_pf));
        } else {
            std::cout << "Begin MVC measurement... " << std::endl;
            mvc_df = -1;
            mvc_pf = 1;
        }
    }
    // Switch between torque offset measurement
    if(set_offset_!=config.set_offset) {
        set_offset_ = config.set_offset;
        if (!set_offset_) {
            // End measurement
            std::cout << std::setprecision(2) << "Torque offset: " << (mvc_offset/n_offset) << std::endl;
            robot_->setTorqueOffset((mvc_offset/n_offset));
        } else {
            std::cout << "Begin offset measurement... " << std::endl;
            mvc_offset = 0;
            n_offset = 1;
        }
    }
    // Change control mode on RQT GUI change
    if(controller_mode_!=config.controller_mode)
    {
        controller_mode_ = config.controller_mode;
        if(controller_mode_ == 0) {
            robot_->initVelocityControl();
            cali_stage = 1;
            cali_velocity = -20;
        }

        cycle = 0;
        counter = 0;

        if (controller_mode_ == 1) robot_->initTorqueControl();
        if (controller_mode_ == 2) robot_->initPositionControl();
        if (controller_mode_ == 3) robot_->initTorqueControl();
        if (controller_mode_ == 4) robot_->initTorqueControl();
        if (controller_mode_ == 5) robot_->initTorqueControl();

        if (controller_mode_ == 6 || controller_mode_ == 7) {
            robot_->initPositionControl();
            fixed_stage_ = 2;
            step_angle_ = 10.0;
        }
        if (controller_mode_ == 8) robot_->initVelocityControl();
        if (controller_mode_ == 10) {
            robot_->initTorqueControl();
            freq = 0.1;
            mag = 2.6;   // magnitude for sine wave (without compensation = 3, with compensation = 0.6)
            s_mag = 0; // current value of sine wave
            s_mag_prev = 0; // previous value of sine wave
            step = 0.1; // step for ramp
            dir = true;
            start = true;
        }
        if (controller_mode_ == 11) time0 = std::chrono::steady_clock::now();
        if (controller_mode_ == 11) robot_->setDigitalOut(0);
        if (controller_mode_ == 12) robot_->initVelocityControl();
    }

    return;
}



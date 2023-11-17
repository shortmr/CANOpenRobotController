/**
 * /file MultiControllerState.h
 * \author Emek Baris Kucuktabak
 * \version 0.1
 * \date 2020-11-09
 * \copyright Copyright (c) 2020
 *
 *
 */

#ifndef SRC_MULTICONTROLLERSTATE_H
#define SRC_MULTICONTROLLERSTATE_H

#include "State.h"
#include "RobotM1.h"
#include "MultiM1MachineROS.h"

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <CORC/dynamic_paramsConfig.h>

// string
#include <string>

/**
 * \brief A multi purpose state with different controllers implemented
 *
 *
 */
class MultiControllerState : public State {
    RobotM1 *robot_;
    MultiM1MachineROS *multiM1MachineRos_;

public:
    void entry(void);
    void during(void);
    void exit(void);
    MultiControllerState(StateMachine *m, RobotM1 *exo, MultiM1MachineROS *multiM1MachineRos, const char *name = NULL) :
                        State(m, name), robot_(exo), multiM1MachineRos_(multiM1MachineRos){};

    // Calibration terms
    int cali_stage;
    int cali_velocity;
    double cali_tau_thresh;
    double cali_vel_thresh;
    double cali_tau_safety;

    // Fixed angle terms
    int fixed_stage;
    double fixed_q; //fixed angular position in degrees

    // Subject-specific settings
    bool set_rom_ = false;
    double rom_df = 0;
    double rom_pf = 0;
    double rom_center = 45;

    bool set_mvc_ = false;
    double mvc_df = 1;
    double mvc_pf = -1;
    double mvc_offset = 0;
    double n_offset = 0;
    bool set_offset_ = false;

    // System identification with torque control
    int cycle; // cycle counter for system identification
    int max_cycle; // number of cycles for system identification
    int id_mode; // mode for system identification (sine = 1; ramp = 2)
    int counter; // loop counter
    double freq; //frequency of sine term
    double mag; //magnitude of sine term
    double s_mag; //current value of sine term
    double s_mag_prev; //previous value of sine term

    double step; // step of ramp function (torque)
    double max_tau; // magnitude of ramp function (torque)
    bool dir; // flag for current direction of ramp function
    bool start; // flag for initial cycle of system identification
    double v_bias; // vertical bias of robot

    double r2d;
    double d2r;

    // Transparency parameters
    double kp_;
    double ki_;
    double kd_;
    double tick_max_;
    double ffRatio_;
    double cut_off_;
    int controller_mode_;

    double control_freq;
    int current_mode;
    double torque_error_last_time_step = 0;

    double error;
    double delta_error;
    double integral_error;

    double spring_tor;
    double tick_count;

    Eigen::VectorXd q; //positive dorsi-flexion
    Eigen::VectorXd dq;
    Eigen::VectorXd tau;
    Eigen::VectorXd tau_s;
    Eigen::VectorXd tau_cmd;

    double alpha_q;
    double alpha_dq;
    double alpha_tau_s;
    double q_pre;
    double tau_pre;
    double tau_raw;
    double tau_filtered;
    double q_raw;
    double q_filtered;
    double dq_raw;
    double dq_filtered;

    // External trigger
    int digitalInValue_;
    int digitalOutValue_;

    RobotParameters m1Params;
private:
    // dynamic reconfigure server and callback
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig> server_;
    void dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level);

    std::chrono::steady_clock::time_point time0;

protected:
    struct timespec initTime;   /*<! Time of state init */
    double lastTime;            /*<! Time of last during() call (in seconds since state init())*/
    double elapsedTime;         /*<! Time since state init() in seconds*/
    double dt;                  /*<! Time between last two during() calls (in seconds)*/
};


#endif //SRC_MULTICONTROLLERSTATE_H

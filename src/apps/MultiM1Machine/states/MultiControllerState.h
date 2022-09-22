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

    // Fixed neutral terms
    int fixed_stage;
    double fixed_q; //neutral position in degrees

    // System identification
    int cycle; // cycle counter for system identification
    int max_cycle; // number of cycles for system identification
    int id_mode; // mode for system identification (sine = 1; ramp = 2)
    int control_mode; // mode for controller during system identification (position = 1; torque = 2)
    int counter; // loop counter
    double freq; //frequency of sine term
    double mag; //magnitude of sine term
    double step; // step of ramp function (torque)
    double max_tau; // magnitude of ramp function (torque)
    double ulim; // lower limit of ramp function (position)
    double llim; // lower limit of ramp function (position)
    bool dir; // flag for current direction of ramp function
    bool start; // flag for initial cycle of system identification

    // Transparency parameters
    double kp_;
    double kd_;
    double ki_;
    double tick_max_;
    double spk_;
    double ffRatio_;
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
    double alpha_tau;
    double q_pre;
    double tau_pre;
    double cut_off;
    double tau_raw;
    double tau_filtered;
    double q_raw;
    double q_filtered;

    // External trigger
    int digitalInValue_;
    int digitalOutValue_;
    std::string m1_trigger = "m1_y";

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

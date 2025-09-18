#ifndef CONTROLLER_H
#define CONTROLLER_H

// -- 1. Data structure to hold the controller's persistent state (integrators) --
typedef struct {
    double theta_int;
    double w_err_int_launch;
    double w_err_int_q;
} ControllerState;

// -- 2. Data structure to hold all controller gains and settings --
typedef struct {
    double l_kp_inner, l_ki_inner;
    double l_kp_outer, l_ki_outer;
    double Q_kp_inner, Q_ki_inner;
    double Q_kp_outer, Q_ki_outer;
    double qbar_max;
} ControllerConfig;

// -- 3. Function Prototypes --
// Declares the functions that will be defined in controller.c

// Initializes the controller state (e.g., resets integrators)
void controller_init(ControllerState* st);

// Executes one step of the controller logic
double controller_step(ControllerState* st, const ControllerConfig* cfg,
                       double Qbar, double w, double theta, double theta_cmd,
                       double Ts);

#endif /* CONTROLLER_H */

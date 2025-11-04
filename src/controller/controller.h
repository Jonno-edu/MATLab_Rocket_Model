#ifndef CONTROLLER_H
#define CONTROLLER_H

typedef struct {
    double w, x, y, z;
} Quaternion;

typedef struct {
    // Outer loop state (attitude errors in non-spinning frame)
    double e_theta_prev;
    double e_psi_prev;
    double e_theta_deriv_filt;  // Filtered derivative for pitch
    double e_psi_deriv_filt;    // Filtered derivative for yaw
    
    // Roll angle control state
    double e_phi_prev;
    double e_phi_deriv_filt;    // Filtered derivative for roll angle error

    // Inner loop state (rate errors in body frame)
    double q_error_int;
    double q_error_prev;
    double q_deriv_filt;        // Filtered derivative for pitch rate
    double r_error_int;
    double r_error_prev;
    double r_deriv_filt;        // Filtered derivative for yaw rate

    // Rate commands
    double q_cmd_ns;
    double r_cmd_ns;
    double q_cmd_body;
    double r_cmd_body;
} ControllerState;

typedef struct {
    // Outer loop gains (non-spinning frame for pitch/yaw)
    double kp_outer_pitch;
    double kd_outer_pitch;
    double kp_outer_yaw;
    double kd_outer_yaw;

    // Inner loop gains (body frame - rate control for pitch/yaw)
    double kp_inner_pitch;
    double ki_inner_pitch;
    double kd_inner_pitch;

    double kp_inner_yaw;
    double ki_inner_yaw;
    double kd_inner_yaw;

    // Roll angle control gains (PD controller)
    double kp_roll;
    double kd_roll;

    // Derivative filter coefficient
    double N;  // Filter coefficient (typically 100)
} ControllerConfig;

void controller_init(ControllerState* st);

void controller_step_multirate(ControllerState* st, const ControllerConfig* cfg,
                               double current_time, double* last_outer_time, double* last_roll_time,
                               Quaternion q_BE, double ref_pitch, double ref_yaw,
                               double q, double r, double roll_cmd,
                               double Ts_inner, double Ts_outer, double Ts_roll,
                               double* Y_nozzle_cmd, double* Z_nozzle_cmd, double* X_roll_cmd);

#endif /* CONTROLLER_H */

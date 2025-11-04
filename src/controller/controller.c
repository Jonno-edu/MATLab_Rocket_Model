#include "controller.h"
#include <math.h>

// Quaternion operations
typedef struct {
    double roll, pitch, yaw;
} EulerAngles;

// Quaternion conjugate
Quaternion quat_conjugate(Quaternion q) {
    return (Quaternion){q.w, -q.x, -q.y, -q.z};
}

// Quaternion multiply
Quaternion quat_multiply(Quaternion q1, Quaternion q2) {
    Quaternion result;
    result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return result;
}

// Euler to quaternion
Quaternion euler_to_quat(double roll, double pitch, double yaw) {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

// Quaternion to Euler
EulerAngles quat_to_euler(Quaternion q) {
    EulerAngles angles;

    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1.0) {
        angles.pitch = copysign(M_PI / 2.0, sinp);
    } else {
        angles.pitch = asin(sinp);
    }

    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}

// Calculate attitude errors in non-spinning frame
void calc_nonspinning_errors(Quaternion q_BE, double ref_pitch, double ref_yaw,
                             double* e_theta, double* e_psi, double* phi) {
    EulerAngles current = quat_to_euler(q_BE);
    *phi = current.roll;

    Quaternion q_BE_nonspinning = euler_to_quat(0.0, current.pitch, current.yaw);
    Quaternion q_ref = euler_to_quat(0.0, ref_pitch, ref_yaw);

    Quaternion q_error = quat_multiply(q_ref, quat_conjugate(q_BE_nonspinning));
    EulerAngles error_angles = quat_to_euler(q_error);

    *e_theta = error_angles.pitch;
    *e_psi = error_angles.yaw;
}

// Rotate pitch rate commands from non-spinning to body frame
void pitch_rate_rotation(double q_cmd_ns, double r_cmd_ns, double phi,
                        double* q_cmd_body, double* r_cmd_body) {
    *q_cmd_body = q_cmd_ns * cos(phi) + r_cmd_ns * sin(phi);
    *r_cmd_body = -q_cmd_ns * sin(phi) + r_cmd_ns * cos(phi);
}

// Outer loop: Attitude errors -> Rate commands (non-spinning frame)
void controller_step_outer(ControllerState* st, const ControllerConfig* cfg,
                          Quaternion q_BE, double ref_pitch, double ref_yaw,
                          double Ts_outer,
                          double* q_cmd_ns, double* r_cmd_ns, double* phi_out) {
    double e_theta, e_psi, phi;
    calc_nonspinning_errors(q_BE, ref_pitch, ref_yaw, &e_theta, &e_psi, &phi);
    *phi_out = phi;

    // Compute raw derivatives
    double e_theta_deriv_raw = (e_theta - st->e_theta_prev) / Ts_outer;
    double e_psi_deriv_raw = (e_psi - st->e_psi_prev) / Ts_outer;

    // Low-pass filter for derivatives: D_filt = (N * D_raw + D_filt_prev) / (1 + N)
    // This is a simple first-order filter with cutoff frequency related to N
    double alpha_outer = cfg->N / (cfg->N + 1.0 / Ts_outer);
    st->e_theta_deriv_filt = alpha_outer * e_theta_deriv_raw + (1.0 - alpha_outer) * st->e_theta_deriv_filt;
    st->e_psi_deriv_filt = alpha_outer * e_psi_deriv_raw + (1.0 - alpha_outer) * st->e_psi_deriv_filt;

    // PD control for pitch channel (using filtered derivative)
    *q_cmd_ns = cfg->kp_outer_pitch * e_theta + cfg->kd_outer_pitch * st->e_theta_deriv_filt;
    st->e_theta_prev = e_theta;

    // PD control for yaw channel (using filtered derivative)
    *r_cmd_ns = cfg->kp_outer_yaw * e_psi + cfg->kd_outer_yaw * st->e_psi_deriv_filt;
    st->e_psi_prev = e_psi;
}

// Inner loop: Rate errors -> Actuator commands (body frame)
void controller_step_inner(ControllerState* st, const ControllerConfig* cfg,
                          double q, double r,
                          double q_cmd_body, double r_cmd_body,
                          double Ts_inner,
                          double* Y_nozzle_cmd, double* Z_nozzle_cmd) {
    // Pitch axis (Y nozzle)
    double q_error = q_cmd_body - q;
    st->q_error_int += q_error * Ts_inner;
    
    // Filtered derivative for pitch
    double q_deriv_raw = (q_error - st->q_error_prev) / Ts_inner;
    double alpha_inner = cfg->N / (cfg->N + 1.0 / Ts_inner);
    st->q_deriv_filt = alpha_inner * q_deriv_raw + (1.0 - alpha_inner) * st->q_deriv_filt;
    
    *Y_nozzle_cmd = cfg->kp_inner_pitch * q_error +
                    cfg->ki_inner_pitch * st->q_error_int +
                    cfg->kd_inner_pitch * st->q_deriv_filt;
    st->q_error_prev = q_error;

    // Yaw axis (Z nozzle)
    double r_error = r_cmd_body - r;
    st->r_error_int += r_error * Ts_inner;
    
    // Filtered derivative for yaw
    double r_deriv_raw = (r_error - st->r_error_prev) / Ts_inner;
    st->r_deriv_filt = alpha_inner * r_deriv_raw + (1.0 - alpha_inner) * st->r_deriv_filt;
    
    *Z_nozzle_cmd = cfg->kp_inner_yaw * r_error +
                    cfg->ki_inner_yaw * st->r_error_int +
                    cfg->kd_inner_yaw * st->r_deriv_filt;
    st->r_error_prev = r_error;
}

// Multi-rate controller wrapper
void controller_step_multirate(ControllerState* st, const ControllerConfig* cfg,
                               double current_time, double* last_outer_time, double* last_roll_time,
                               Quaternion q_BE, double ref_pitch, double ref_yaw,
                               double q, double r, double roll_cmd,
                               double Ts_inner, double Ts_outer, double Ts_roll,
                               double* Y_nozzle_cmd, double* Z_nozzle_cmd, double* X_roll_cmd) {
    // Run outer loop at slower rate
    if ((current_time - *last_outer_time) >= Ts_outer) {
        double phi;
        controller_step_outer(st, cfg, q_BE, ref_pitch, ref_yaw, Ts_outer,
                            &st->q_cmd_ns, &st->r_cmd_ns, &phi);

        pitch_rate_rotation(st->q_cmd_ns, st->r_cmd_ns, phi,
                          &st->q_cmd_body, &st->r_cmd_body);

        *last_outer_time = current_time;
    }

    // Inner loop runs every call (pitch and yaw rate control)
    controller_step_inner(st, cfg, q, r,
                         st->q_cmd_body, st->r_cmd_body,
                         Ts_inner,
                         Y_nozzle_cmd, Z_nozzle_cmd);
    
    // Roll angle control (PD controller) runs at its own rate
    // Always initialize to prevent garbage values
    *X_roll_cmd = 0.0;
    
    if ((current_time - *last_roll_time) >= Ts_roll) {
        EulerAngles angles = quat_to_euler(q_BE);
        double roll_error = roll_cmd - angles.roll;
        
        // Filtered derivative for roll
        double roll_deriv_raw = (roll_error - st->e_phi_prev) / Ts_roll;
        double alpha = cfg->N / (cfg->N + 1.0 / Ts_roll);
        st->e_phi_deriv_filt = alpha * roll_deriv_raw + (1.0 - alpha) * st->e_phi_deriv_filt;
        
        *X_roll_cmd = cfg->kp_roll * roll_error + cfg->kd_roll * st->e_phi_deriv_filt;
        st->e_phi_prev = roll_error;

        *last_roll_time = current_time;
    }
}

void controller_init(ControllerState* st) {
    st->e_theta_prev = 0.0;
    st->e_psi_prev = 0.0;
    st->e_theta_deriv_filt = 0.0;
    st->e_psi_deriv_filt = 0.0;
    st->e_phi_prev = 0.0;
    st->e_phi_deriv_filt = 0.0;
    st->q_error_int = 0.0;
    st->q_error_prev = 0.0;
    st->q_deriv_filt = 0.0;
    st->r_error_int = 0.0;
    st->r_error_prev = 0.0;
    st->r_deriv_filt = 0.0;
    st->q_cmd_ns = 0.0;
    st->r_cmd_ns = 0.0;
    st->q_cmd_body = 0.0;
    st->r_cmd_body = 0.0;
}

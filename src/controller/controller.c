#include "controller.h" // Include our own header to ensure consistency

//
// Implementation of the controller_init function
//
void controller_init(ControllerState* st)
{
    // Reset all integral terms to zero at the start of a simulation
    st->theta_int = 0.0;
    st->w_err_int_launch = 0.0;
    st->w_err_int_q = 0.0;
}

//
// Implementation of the controller_step function
//
double controller_step(ControllerState* st, const ControllerConfig* cfg,
                       double Qbar, double w, double theta, double theta_cmd,
                       double Ts)
{
    // --- Outer Loop: Pitch Error -> Pitch Rate Command ---
    const double theta_error = theta_cmd - theta;
    st->theta_int += theta_error * Ts; // Update pitch error integral

    // Calculate the target pitch rate for both the launch and q-blend controllers
    const double w_cmd_launch = cfg->l_kp_outer * theta_error + cfg->l_ki_outer * st->theta_int;
    const double w_cmd_q      = cfg->Q_kp_outer * theta_error + cfg->Q_ki_outer * st->theta_int;

    // --- Inner Loop: Pitch Rate Error -> Actuator Command ---
    const double w_err_launch = w_cmd_launch - w;
    const double w_err_q      = w_cmd_q      - w;

    // Update the inner loop integrals
    st->w_err_int_launch += w_err_launch * Ts;
    st->w_err_int_q      += w_err_q * Ts;

    // Calculate the final actuator command for both controllers
    const double act_cmd_launch = cfg->l_kp_inner * w_err_launch + cfg->l_ki_inner * st->w_err_int_launch;
    const double act_cmd_q      = cfg->Q_kp_inner * w_err_q      + cfg->Q_ki_inner * st->w_err_int_q;

    // --- Controller Blending ---
    // Calculate the blend factor based on dynamic pressure (qbar)
    double w_blend = Qbar / cfg->qbar_max;

    // Clamp the blend factor to be between 0 and 1
    if (w_blend < 0.0) {
        w_blend = 0.0;
    } else if (w_blend > 1.0) {
        w_blend = 1.0;
    }

    // Linearly interpolate between the two controller outputs
    const double final_actuator_cmd = (1.0 - w_blend) * act_cmd_launch + w_blend * act_cmd_q;
    
    return final_actuator_cmd;
}

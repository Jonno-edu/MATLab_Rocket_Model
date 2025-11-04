#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#include "controller.h"

/*
 * Default Controller Configuration
 * These gains are tuned for the rocket TVC system.
 * Inner loop runs at 100 Hz, outer loop at 20 Hz.
 */

// --- INNER LOOP (Rate Control, 100 Hz) ---
#define DEFAULT_KP_INNER_PITCH  1.561224
#define DEFAULT_KI_INNER_PITCH  9.593220
#define DEFAULT_KD_INNER_PITCH  0.0      // PI only

#define DEFAULT_KP_INNER_YAW    1.561224
#define DEFAULT_KI_INNER_YAW    9.593220
#define DEFAULT_KD_INNER_YAW    0.0      // PI only

#define DEFAULT_KP_ROLL         40.0     // Placeholder - tune as needed
#define DEFAULT_KD_ROLL         200.0    // Placeholder - tune as needed

// --- OUTER LOOP (Attitude Control, 20 Hz) ---
#define DEFAULT_KP_OUTER_PITCH  2.487179
#define DEFAULT_KD_OUTER_PITCH  0.215517

#define DEFAULT_KP_OUTER_YAW    2.487179
#define DEFAULT_KD_OUTER_YAW    0.215517

// --- SAMPLE TIMES ---
#define DEFAULT_TS_INNER        0.005     // 200 Hz
#define DEFAULT_TS_OUTER        0.025     // 40 Hz
#define DEFAULT_TS_ROLL         0.02      // 50 Hz

// --- DERIVATIVE FILTER ---
#define DEFAULT_N               100.0     // Filter coefficient for derivative terms

// Helper function to initialize a ControllerConfig with default gains
static inline ControllerConfig get_default_controller_config(void) {
    ControllerConfig cfg;
    
    // Outer loop gains (non-spinning frame)
    cfg.kp_outer_pitch = DEFAULT_KP_OUTER_PITCH;
    cfg.kd_outer_pitch = DEFAULT_KD_OUTER_PITCH;
    cfg.kp_outer_yaw = DEFAULT_KP_OUTER_YAW;
    cfg.kd_outer_yaw = DEFAULT_KD_OUTER_YAW;
    
    // Inner loop gains (body frame)
    cfg.kp_inner_pitch = DEFAULT_KP_INNER_PITCH;
    cfg.ki_inner_pitch = DEFAULT_KI_INNER_PITCH;
    cfg.kd_inner_pitch = DEFAULT_KD_INNER_PITCH;
    
    cfg.kp_inner_yaw = DEFAULT_KP_INNER_YAW;
    cfg.ki_inner_yaw = DEFAULT_KI_INNER_YAW;
    cfg.kd_inner_yaw = DEFAULT_KD_INNER_YAW;

    // Roll angle control gains
    cfg.kp_roll = DEFAULT_KP_ROLL;
    cfg.kd_roll = DEFAULT_KD_ROLL;

    // Derivative filter coefficient
    cfg.N = DEFAULT_N;
    
    return cfg;
}

#endif /* CONTROLLER_CONFIG_H */

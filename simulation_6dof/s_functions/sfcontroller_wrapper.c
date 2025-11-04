#define S_FUNCTION_NAME  sfcontroller_wrapper
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "mex.h"
#include "controller.h"
#include "controller_config.h"

// S-Function Parameters (none - all configuration from header file)
enum { 
    NUM_PARAMS = 0
};

// DWork Vectors
enum { 
    DW_STATE = 0,
    DW_CONFIG = 1,
    DW_LAST_OUTER_TIME = 2,
    DW_LAST_ROLL_TIME = 3,
    NUM_DWORK 
};

// Input port indices
enum {
    IN_QUAT = 0,        // Quaternion [qw, qx, qy, qz] - 4 element vector
    IN_BODY_RATES = 1,  // Body rates [p, q, r] - 3 element vector
    IN_REF_PITCH = 2,   // Reference pitch angle (scalar)
    IN_REF_YAW = 3,     // Reference yaw angle (scalar)
    IN_ROLL_CMD = 4,    // Roll angle command (scalar) - NOT rate
    NUM_INPUTS = 5
};

// Output port indices
enum {
    OUT_NOZZLE_CMDS = 0,  // [Y_nozzle, Z_nozzle, X_roll] - 3 element vector
    NUM_OUTPUTS = 1
};

/*================*
 * S-function methods *
 *================*/

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    // Configure Input Ports
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;
    
    // Quaternion input (4-element vector)
    ssSetInputPortWidth(S, IN_QUAT, 4);
    ssSetInputPortDirectFeedThrough(S, IN_QUAT, 1);
    ssSetInputPortRequiredContiguous(S, IN_QUAT, 1);
    
    // Body rates input (3-element vector)
    ssSetInputPortWidth(S, IN_BODY_RATES, 3);
    ssSetInputPortDirectFeedThrough(S, IN_BODY_RATES, 1);
    ssSetInputPortRequiredContiguous(S, IN_BODY_RATES, 1);
    
    // Reference pitch (scalar)
    ssSetInputPortWidth(S, IN_REF_PITCH, 1);
    ssSetInputPortDirectFeedThrough(S, IN_REF_PITCH, 1);
    ssSetInputPortRequiredContiguous(S, IN_REF_PITCH, 1);
    
    // Reference yaw (scalar)
    ssSetInputPortWidth(S, IN_REF_YAW, 1);
    ssSetInputPortDirectFeedThrough(S, IN_REF_YAW, 1);
    ssSetInputPortRequiredContiguous(S, IN_REF_YAW, 1);
    
    // Roll angle command (scalar)
    ssSetInputPortWidth(S, IN_ROLL_CMD, 1);
    ssSetInputPortDirectFeedThrough(S, IN_ROLL_CMD, 1);
    ssSetInputPortRequiredContiguous(S, IN_ROLL_CMD, 1);

    // Configure Output Port (3-element vector)
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    ssSetOutputPortWidth(S, OUT_NOZZLE_CMDS, 3);

    ssSetNumSampleTimes(S, 1);

    // DWork vectors
    ssSetNumDWork(S, NUM_DWORK);
    
    ssSetDWorkWidth(S, DW_STATE, (int_T)sizeof(ControllerState));
    ssSetDWorkDataType(S, DW_STATE, SS_UINT8);
    ssSetDWorkName(S, DW_STATE, "ControllerState");
    
    ssSetDWorkWidth(S, DW_CONFIG, (int_T)sizeof(ControllerConfig));
    ssSetDWorkDataType(S, DW_CONFIG, SS_UINT8);
    ssSetDWorkName(S, DW_CONFIG, "ControllerConfig");
    
    ssSetDWorkWidth(S, DW_LAST_OUTER_TIME, 1);
    ssSetDWorkDataType(S, DW_LAST_OUTER_TIME, SS_DOUBLE);
    ssSetDWorkName(S, DW_LAST_OUTER_TIME, "LastOuterTime");

    ssSetDWorkWidth(S, DW_LAST_ROLL_TIME, 1);
    ssSetDWorkDataType(S, DW_LAST_ROLL_TIME, SS_DOUBLE);
    ssSetDWorkName(S, DW_LAST_ROLL_TIME, "LastRollTime");
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    // Use sample time from configuration header
    ssSetSampleTime(S, 0, DEFAULT_TS_INNER);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    ControllerState* state = (ControllerState*)ssGetDWork(S, DW_STATE);
    ControllerConfig* config = (ControllerConfig*)ssGetDWork(S, DW_CONFIG);
    real_T* last_outer_time = (real_T*)ssGetDWork(S, DW_LAST_OUTER_TIME);
    real_T* last_roll_time = (real_T*)ssGetDWork(S, DW_LAST_ROLL_TIME);
    
    controller_init(state);
    *config = get_default_controller_config();
    *last_outer_time = 0.0;
    *last_roll_time = 0.0;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Get vector inputs
    const real_T* quat = (const real_T*)ssGetInputPortSignal(S, IN_QUAT);
    const real_T* body_rates = (const real_T*)ssGetInputPortSignal(S, IN_BODY_RATES);
    
    // Get scalar inputs
    const real_T ref_pitch = *(const real_T*)ssGetInputPortSignal(S, IN_REF_PITCH);
    const real_T ref_yaw = *(const real_T*)ssGetInputPortSignal(S, IN_REF_YAW);
    const real_T roll_cmd = *(const real_T*)ssGetInputPortSignal(S, IN_ROLL_CMD);
    
    // Get vector output
    real_T* output = (real_T*)ssGetOutputPortSignal(S, OUT_NOZZLE_CMDS);
    
    // Get persistent data
    ControllerState* state = (ControllerState*)ssGetDWork(S, DW_STATE);
    ControllerConfig* config = (ControllerConfig*)ssGetDWork(S, DW_CONFIG);
    real_T* last_outer_time = (real_T*)ssGetDWork(S, DW_LAST_OUTER_TIME);
    real_T* last_roll_time = (real_T*)ssGetDWork(S, DW_LAST_ROLL_TIME);
    
    // Use sample times from configuration header
    const real_T Ts_inner = DEFAULT_TS_INNER;
    const real_T Ts_outer = DEFAULT_TS_OUTER;
    const real_T Ts_roll = DEFAULT_TS_ROLL;
    
    // Get current simulation time
    time_T current_time = ssGetT(S);
    
    // Create quaternion struct
    Quaternion q_BE = {quat[0], quat[1], quat[2], quat[3]};
    
    // Extract body rates
    const real_T q = body_rates[1];
    const real_T r = body_rates[2];
    
    // Call multi-rate controller
    controller_step_multirate(state, config,
                             current_time, last_outer_time, last_roll_time,
                             q_BE, ref_pitch, ref_yaw,
                             q, r, roll_cmd,
                             Ts_inner, Ts_outer, Ts_roll,
                             &output[0], &output[1], &output[2]);
}

static void mdlTerminate(SimStruct *S)
{
    (void)S;
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

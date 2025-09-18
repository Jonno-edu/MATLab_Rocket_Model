#define S_FUNCTION_NAME  sfcontroller_wrapper
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "mex.h"

#include "controller.h" 

/*
 * S-Function Parameters
 * We expect one parameter from the dialog box: the sample time (Ts).
 */
enum { P_TS = 0, NUM_PARAMS };

/*
 * DWork Vectors
 * We need one "work" vector to store the persistent state of our controller.
 * We will treat this memory block as our ControllerState struct.
 */
enum { DW_STATE = 0, NUM_DWORK };

/*
 * This is the hardcoded configuration for your controller gains.
 * In a more advanced version, you could pass these in as S-Function parameters.
 */
static const ControllerConfig kControllerConfig = {
    // Inner loop gains (launch)
    0.0671, 0.0119, 
    // Outer loop gains (launch)
    0.0813, 0.0041, 
    // Inner loop gains (q-blend)
    0.9925, 1.5251, 
    // Outer loop gains (q-blend)
    0.7421, 0.0156, 
    // Max dynamic pressure for blending
    78000.0 * 1.2
};


/*================*
 * S-function methods *
 *================*/

//
// mdlInitializeSizes: Tell Simulink about the block's ports, parameters, etc.
//
static void mdlInitializeSizes(SimStruct *S)
{
    // Set the number of expected dialog parameters (just Ts)
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; // Mismatch in parameter count
    }

    // Configure Input Ports: We need 4 scalar inputs.
    if (!ssSetNumInputPorts(S, 4)) return;
    for (int_T i = 0; i < 4; ++i) {
        ssSetInputPortWidth(S, i, 1); // Each port is a scalar
        ssSetInputPortDirectFeedThrough(S, i, 1); // Output depends on these inputs
        ssSetInputPortRequiredContiguous(S, i, 1); // For efficient memory access
    }

    // Configure Output Ports: We have 1 scalar output.
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);

    // Sample Time: We have one discrete sample time, set by our parameter.
    ssSetNumSampleTimes(S, 1);

    // DWork: Allocate memory for our controller's state.
    // We allocate it as a block of bytes, matching the size of our struct.
    ssSetNumDWork(S, NUM_DWORK);
    ssSetDWorkWidth(S, DW_STATE, (int_T)sizeof(ControllerState));
    ssSetDWorkDataType(S, DW_STATE, SS_UINT8); // Treat as raw bytes
    ssSetDWorkName(S, DW_STATE, "ControllerState"); // Give it a name for debugging
}


//
// mdlInitializeSampleTimes: Set the sample time from the dialog parameter.
//
static void mdlInitializeSampleTimes(SimStruct *S)
{
    // Get the sample time (Ts) from the first S-Function parameter (P_TS)
    real_T Ts = mxGetScalar(ssGetSFcnParam(S, P_TS));
    ssSetSampleTime(S, 0, Ts);
    ssSetOffsetTime(S, 0, 0.0);
}


//
// mdlStart: Called once at the beginning of the simulation.
// This is the perfect place to call our controller_init function.
//
#define MDL_START
static void mdlStart(SimStruct *S)
{
    // Get a pointer to the DWork memory block
    ControllerState* state = (ControllerState*)ssGetDWork(S, DW_STATE);
    
    // Call the external initialization function from controller.c
    controller_init(state);
}


//
// mdlOutputs: Called at every time step to calculate the block's output.
//
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // --- Get pointers to the inputs ---
    const real_T Qbar      = *(const real_T*) ssGetInputPortSignal(S, 0);
    const real_T w         = *(const real_T*) ssGetInputPortSignal(S, 1);
    const real_T theta     = *(const real_T*) ssGetInputPortSignal(S, 2);
    const real_T theta_cmd = *(const real_T*) ssGetInputPortSignal(S, 3);

    // --- Get a pointer to the output ---
    real_T *y_actuator_cmd = (real_T*) ssGetOutputPortSignal(S, 0);

    // --- Get a pointer to our controller's persistent state ---
    ControllerState* state = (ControllerState*)ssGetDWork(S, DW_STATE);
    
    // Get the sample time for this step
    const real_T Ts = ssGetSampleTime(S, 0);

    // --- The Core Logic: Call the external controller ---
    // This single function call does all the work. The implementation is
    // entirely in controller.c, which is shared with the Pico project.
    y_actuator_cmd[0] = controller_step(state, &kControllerConfig, 
                                        Qbar, w, theta, theta_cmd, Ts);
}


//
// mdlTerminate: Called at the end of the simulation for cleanup.
//
static void mdlTerminate(SimStruct *S)
{
    // Nothing to do for this simple controller
    (void)S; // Suppress unused parameter warning
}


/*=============================*
 * Required S-function trailer *
 *=============================*/
#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

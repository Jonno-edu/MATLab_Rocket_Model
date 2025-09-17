// Defines the name of the S-function for compilation
#define S_FUNCTION_NAME  mulSFunction
#define S_FUNCTION_LEVEL 2

// Required header for S-Function structure
#include "simstruc.h"

/*
 * mdlInitializeSizes: Set up the number of input and output ports,
 * their widths, and other basic properties of the S-Function.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    // This S-Function has no parameters
    ssSetNumSFcnParams(S, 0);

    // Check for parameter count mismatch
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; // Mismatch will be reported by Simulink
    }

    // --- Input Port Configuration ---
    // This block has one input port
    if (!ssSetNumInputPorts(S, 1)) return;
    // The input port is a scalar (width = 1)
    ssSetInputPortWidth(S, 0, 1);
    // The output depends directly on the input at the current time step
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    // --- Output Port Configuration ---
    // This block has one output port
    if (!ssSetNumOutputPorts(S, 1)) return;
    // The output port is a scalar (width = 1)
    ssSetOutputPortWidth(S, 0, 1);

    // This block has one sample time
    ssSetNumSampleTimes(S, 1);

    // Reserve memory for any other options
    ssSetOptions(S, 0);
}

/*
 * mdlInitializeSampleTimes: Specify the sample time for the block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    // Set the block's sample time to be inherited from the driving block
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

/*
 * mdlOutputs: This is where the core logic of the S-Function is executed.
 * It is called at each simulation time step.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Get a pointer to the input signal
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    // Dereference the pointer to get the input value
    real_T u = *uPtrs[0];

    // Get a pointer to the output signal
    real_T *y = ssGetOutputPortRealSignal(S, 0);

    // --- Core Logic: Multiply the input by 2 ---
    y[0] = -5.0 * u;
}

/*
 * mdlTerminate: This function is called at the end of the simulation.
 * It's used for cleanup (e.g., freeing memory or closing files).
 */
static void mdlTerminate(SimStruct *S)
{
    // No cleanup needed for this simple example.
}

// Required S-Function trailer
#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

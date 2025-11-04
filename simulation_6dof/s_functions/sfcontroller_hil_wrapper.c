#define S_FUNCTION_NAME  sfcontroller_hil_wrapper
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "mex.h"
#include "hil_protocol.h"
#include <string.h>

// S-Function Parameters
enum { 
    P_SERIAL_PORT = 0,  // Serial port name (e.g., "/dev/tty.usbmodem11401")
    P_TIMEOUT_MS = 1,   // Timeout in milliseconds
    NUM_PARAMS 
};

// DWork Vectors
enum { 
    DW_SERIAL_FD = 0,   // Serial port file descriptor
    NUM_DWORK 
};

// Input port indices (same as software controller)
enum {
    IN_QUAT = 0,        // Quaternion [qw, qx, qy, qz] - 4 element vector
    IN_BODY_RATES = 1,  // Body rates [p, q, r] - 3 element vector
    IN_REF_PITCH = 2,   // Reference pitch angle (scalar)
    IN_REF_YAW = 3,     // Reference yaw angle (scalar)
    IN_ROLL_CMD = 4,    // Roll angle command (scalar)
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
    
    ssSetInputPortWidth(S, IN_QUAT, 4);
    ssSetInputPortDirectFeedThrough(S, IN_QUAT, 1);
    ssSetInputPortRequiredContiguous(S, IN_QUAT, 1);
    
    ssSetInputPortWidth(S, IN_BODY_RATES, 3);
    ssSetInputPortDirectFeedThrough(S, IN_BODY_RATES, 1);
    ssSetInputPortRequiredContiguous(S, IN_BODY_RATES, 1);
    
    ssSetInputPortWidth(S, IN_REF_PITCH, 1);
    ssSetInputPortDirectFeedThrough(S, IN_REF_PITCH, 1);
    ssSetInputPortRequiredContiguous(S, IN_REF_PITCH, 1);
    
    ssSetInputPortWidth(S, IN_REF_YAW, 1);
    ssSetInputPortDirectFeedThrough(S, IN_REF_YAW, 1);
    ssSetInputPortRequiredContiguous(S, IN_REF_YAW, 1);
    
    ssSetInputPortWidth(S, IN_ROLL_CMD, 1);
    ssSetInputPortDirectFeedThrough(S, IN_ROLL_CMD, 1);
    ssSetInputPortRequiredContiguous(S, IN_ROLL_CMD, 1);

    // Configure Output Port
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    ssSetOutputPortWidth(S, OUT_NOZZLE_CMDS, 3);

    ssSetNumSampleTimes(S, 1);

    // DWork for serial file descriptor
    ssSetNumDWork(S, NUM_DWORK);
    ssSetDWorkWidth(S, DW_SERIAL_FD, 1);
    ssSetDWorkDataType(S, DW_SERIAL_FD, SS_INT32);
    ssSetDWorkName(S, DW_SERIAL_FD, "SerialFD");
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    // Run at 200 Hz (inner loop rate)
    ssSetSampleTime(S, 0, 0.005);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    // Get serial port name from parameter
    char port_name[256];
    mxGetString(ssGetSFcnParam(S, P_SERIAL_PORT), port_name, sizeof(port_name));
    
    // Open serial port (platform-specific code would go here)
    // For now, store -1 to indicate not connected
    int32_T* serial_fd = (int32_T*)ssGetDWork(S, DW_SERIAL_FD);
    *serial_fd = -1;
    
    // TODO: Implement serial port opening
    // On macOS/Linux: open() with appropriate termios settings
    // On Windows: CreateFile() with DCB settings
    
    mexPrintf("HIL S-Function: Attempting to connect to %s\n", port_name);
    mexPrintf("HIL S-Function: Serial communication not yet implemented\n");
    mexPrintf("HIL S-Function: This is a placeholder - outputs will be zero\n");
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Get inputs
    const real_T* quat = (const real_T*)ssGetInputPortSignal(S, IN_QUAT);
    const real_T* body_rates = (const real_T*)ssGetInputPortSignal(S, IN_BODY_RATES);
    const real_T ref_pitch = *(const real_T*)ssGetInputPortSignal(S, IN_REF_PITCH);
    const real_T ref_yaw = *(const real_T*)ssGetInputPortSignal(S, IN_REF_YAW);
    const real_T roll_cmd = *(const real_T*)ssGetInputPortSignal(S, IN_ROLL_CMD);
    
    // Get output
    real_T* output = (real_T*)ssGetOutputPortSignal(S, OUT_NOZZLE_CMDS);
    
    // Get serial file descriptor
    int32_T* serial_fd = (int32_T*)ssGetDWork(S, DW_SERIAL_FD);
    
    // Get current time
    time_T current_time = ssGetT(S);
    
    // Prepare input packet (11 floats = 44 bytes)
    HIL_InputPacket input_packet;
    input_packet.current_time = (float)current_time;
    input_packet.qw = (float)quat[0];
    input_packet.qx = (float)quat[1];
    input_packet.qy = (float)quat[2];
    input_packet.qz = (float)quat[3];
    input_packet.p = (float)body_rates[0];
    input_packet.q = (float)body_rates[1];
    input_packet.r = (float)body_rates[2];
    input_packet.ref_pitch = (float)ref_pitch;
    input_packet.ref_yaw = (float)ref_yaw;
    input_packet.roll_cmd = (float)roll_cmd;
    
    // TODO: Send packet to Pico and receive response
    // For now, just output zeros
    if (*serial_fd >= 0) {
        // Write HIL_INPUT_SIZE bytes (44 bytes)
        // Read HIL_OUTPUT_SIZE bytes (12 bytes)
        // Extract actuator commands
    } else {
        output[0] = 0.0;  // Y_nozzle
        output[1] = 0.0;  // Z_nozzle
        output[2] = 0.0;  // X_roll
    }
}

static void mdlTerminate(SimStruct *S)
{
    int32_T* serial_fd = (int32_T*)ssGetDWork(S, DW_SERIAL_FD);
    
    if (*serial_fd >= 0) {
        // TODO: Close serial port
        mexPrintf("HIL S-Function: Closing serial connection\n");
        *serial_fd = -1;
    }
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

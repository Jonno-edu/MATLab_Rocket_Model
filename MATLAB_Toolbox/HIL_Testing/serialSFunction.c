// Defines the name of the S-function for compilation
#define S_FUNCTION_NAME  serialSFunction
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
// Headers for C standard libraries and serial communication
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
// Function Prototypes
static int open_serial(SimStruct *S);
static void close_serial(int fd);
/*
 * mdlInitializeSizes: Define S-Function properties
 */
static void mdlInitializeSizes(SimStruct *S)
{
    // --- Define one S-Function parameter for the port name ---
    ssSetNumSFcnParams(S, 1);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) { return; }
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumPWork(S, 1);
    ssSetOptions(S, 0);
}
/*
 * mdlInitializeSampleTimes: Set the execution rate to 100Hz
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.01); // 100 Hz
    ssSetOffsetTime(S, 0, 0.0);
}
/*
 * mdlStart: Called once at the beginning of the simulation
 */
#define MDL_START
static void mdlStart(SimStruct *S)
{
    int fd = open_serial(S);
    ssGetPWork(S)[0] = (void *)(intptr_t)fd;
}
/*
 * mdlOutputs: Called at each 100Hz time step
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    int serial_port = (int)(intptr_t)ssGetPWork(S)[0];
    if (serial_port < 0) { return; }
    
    InputRealPtrsType uPtrs = (InputRealPtrsType)ssGetInputPortSignalPtrs(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    char write_buffer[32];
    char read_buffer[128];
    // Write, wait, and read
    int n_written = sprintf(write_buffer, "%f\n", *uPtrs[0]);
    if (write(serial_port, write_buffer, n_written) < 0) { return; }
    
    usleep(1000); // Reduce wait time for faster loops
    
    memset(read_buffer, 0, sizeof(read_buffer));
    int bytes_read = read(serial_port, read_buffer, sizeof(read_buffer) - 1);
    
    if (bytes_read > 0) {
        y[0] = atof(read_buffer);
    }
}
/*
 * mdlTerminate: Called once at the end of the simulation
 */
static void mdlTerminate(SimStruct *S)
{
    int fd = (int)(intptr_t)ssGetPWork(S)[0];
    close_serial(fd);
}
// --- Helper Functions ---
static int open_serial(SimStruct *S) {
    // --- Get the port name from the S-Function parameter ---
    #define MAX_PORT_NAME_LEN 128
    char port_name[MAX_PORT_NAME_LEN];
    mxGetString(ssGetSFcnParam(S, 0), port_name, MAX_PORT_NAME_LEN);
    
    int fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { return -1; }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if(tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }
    cfsetispeed(&tty, B115200); cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD); tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; tty.c_cflag &= ~PARENB; tty.c_cflag &= ~CSTOPB;
    tty.c_lflag &= ~ICANON; tty.c_lflag &= ~ECHO;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}
static void close_serial(int fd) {
    if (fd > 0) { close(fd); }
}
// Required S-Function trailer
#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
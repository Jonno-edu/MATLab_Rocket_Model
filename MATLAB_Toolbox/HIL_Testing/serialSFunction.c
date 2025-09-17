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
#include <sys/time.h> // For gettimeofday()

// Function Prototypes
static int open_serial(void);
static void close_serial(int fd);
static double get_current_time_ms(void);

/*
 * mdlInitializeSizes: Define S-Function properties
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) { return; }

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    // --- We now have 3 output ports ---
    if (!ssSetNumOutputPorts(S, 3)) return;
    
    // Port 0: Data from Pico
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);

    // Port 1: "Tick" timestamp
    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);

    // Port 2: "Tock" timestamp
    ssSetOutputPortWidth(S, 2, 1);
    ssSetOutputPortDataType(S, 2, SS_DOUBLE);
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumPWork(S, 1); // To store the serial file descriptor
    ssSetOptions(S, 0);
}

/*
 * mdlInitializeSampleTimes: Set the execution rate to 100Hz
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.01); // 0.01 seconds = 100 Hz
    ssSetOffsetTime(S, 0, 0.0);
}

/*
 * mdlStart: Called once at the beginning of the simulation
 */
#define MDL_START
static void mdlStart(SimStruct *S)
{
    int fd = open_serial();
    ssGetPWork(S)[0] = (void *)(intptr_t)fd;
}

/*
 * mdlOutputs: Called at each 100Hz time step
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    int serial_port = (int)(intptr_t)ssGetPWork(S)[0];
    
    if (serial_port < 0) {
        return;
    }
    
    InputRealPtrsType uPtrs = (InputRealPtrsType)ssGetInputPortSignalPtrs(S, 0);
    char write_buffer[32];
    char read_buffer[128];

    // Get pointers to all three output ports
    real_T *y_data = ssGetOutputPortRealSignal(S, 0);
    real_T *y_tick = ssGetOutputPortRealSignal(S, 1);
    real_T *y_tock = ssGetOutputPortRealSignal(S, 2);

    // --- 1. Get "tick" time and Write Data ---
    y_tick[0] = get_current_time_ms();
    
    int n_written = sprintf(write_buffer, "%f\n", *uPtrs[0]);
    if (write(serial_port, write_buffer, n_written) < 0) {
        return;
    }
    
    // --- 2. Wait for Response ---
    memset(read_buffer, 0, sizeof(read_buffer));
    int buffer_pos = 0;
    int max_attempts = 80;
    
    for (int i = 0; i < max_attempts; i++) {
        int bytes_read = read(serial_port, &read_buffer[buffer_pos], 1);
        if (bytes_read > 0) {
            if (read_buffer[buffer_pos] == '\n') {
                break;
            }
            buffer_pos++;
        } else {
            usleep(10); 
        }
    }
    
    // --- 3. Get "tock" time and Update Output ---
    y_tock[0] = get_current_time_ms();

    if (buffer_pos > 0) {
        y_data[0] = atof(read_buffer);
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
static double get_current_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec * 1000.0 + (double)tv.tv_usec / 1000.0;
}

static int open_serial(void) {
    const char *port_name = "/dev/tty.usbmodem1401"; 
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

/*------------------------------------------------*/
/* SimuLink S-Function Template                   */
/*------------------------------------------------*/

// "sftemplate.c" is a template for creating S-Functions in Simulink.
// It provides a basic structure for defining an S-Function with inputs and outputs.
// The template includes necessary headers, defines constants, and declares functions for initialization, outputs, updates, derivatives, and termination.
// It is designed to be modified according to specific requirements of the S-Function being created.
// Replace "sftemplate" with the desired name for your S-Function.
// This filename must match the S_FUNCTION_NAME defined below.
// The S-Function can be used in Simulink models to perform custom computations or simulations.

#define S_FUNCTION_NAME sftemplate

#define PAR0(S) ssGetSFcnParam(S,0)
#define PAR1(S) ssGetSFcnParam(S,1)
#define PAR2(S) ssGetSFcnParam(S,2)
#define PAR3(S) ssGetSFcnParam(S,3)
#define PAR4(S) ssGetSFcnParam(S,4)
#define PAR5(S) ssGetSFcnParam(S,5)
#define PAR6(S) ssGetSFcnParam(S,6)
#define PAR7(S) ssGetSFcnParam(S,7)

#include "tmwtypes.h"
#include "simstruc.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>
#include "mex.h"
// #include <time.h>
/* Inlcude other libraries */

/* Numerical constants */
#define PI		3.14159265358979
#define E6A		1.0E-6
#define PIO2	(0.5*PI)
#define PI2		(2.0*PI)
#define RAD		(PI/180.0)
#define DEG		(180.0/PI)
#define AU	    1.49597870E8   /* Astronomical unit - kilometers (IAU 76) */

double TS;

double actuator_cmd, vel_error, prev_vel_error;
double kp = 10;
double ki = 0;
double kd = 3;

/*************************
 * Define functions here *
 *************************/




/*====================*
 * S-function methods *
 *====================*/

static void mdlInitializeSizes(SimStruct *S)
{ // This function initializes the sizes of the S-function
    /* Set the number of inputs and outputs at
    ssSetNumInputs() and ssSetNumOutputs */
    ssSetNumSFcnParams(S, 1);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(    S, 0);   /* number of continuous states           */
    ssSetNumDiscStates(    S, 0);   /* number of discrete states             */
    ssSetNumInputs(        S, 1);  /* number of inputs                      */
    ssSetNumOutputs(       S, 1);  /* number of outputs                     */
    ssSetDirectFeedThrough(S, 1);   /* direct feedthrough flag               */
    ssSetNumSampleTimes(   S, 1);   /* number of sample times                */
    ssSetNumRWork(         S, 0);   /* number of real work vector elements   */
    ssSetNumIWork(         S, 0);   /* number of integer work vector elements*/
    ssSetNumPWork(         S, 0);   /* number of pointer work vector elements*/
    ssSetNumModes(         S, 0);   /* number of mode work vector elements   */
    ssSetNumNonsampledZCs( S, 0);   /* number of nonsampled zero crossings   */
    ssSetOptions(          S, 0);   /* general options (SS_OPTION_xx)        */
}

static void mdlInitializeSampleTimes(SimStruct *S)
{ // This function initializes the sample times for the sfunction
 	TS = mxGetScalar(PAR0(S));
    ssSetSampleTime(S, 0, TS);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlInitializeConditions(real_T *x0, SimStruct *S)
{ // You can leave this empty if no initial conditions are needed
    /* This function is called at the start of the simulation to initialize states */
    /* Example: Initialize the first state to zero */
    // x0[0] = 0.0;  // Set first state to zero
    prev_vel_error = 0;

    // leave empty    
}

static void mdlOutputs(real_T *y, const real_T *x, const real_T *u, 
                       SimStruct *S, int_T tid)
{ // This function is called at each time step to compute the outputs
    /*  u is an array of the inputs 
        y is an array of the outputs 
        
        Use your inputs as u[0]...u[i] and set your outputs as y[0]...y[i]

        Do your operations inbetween reading the inputs and setting the outputs.
    */
    /* Example: Set the first output to the first input */
    // y[0] = u[0];  // Copy first input to first output
    vel_error = u[0];

    actuator_cmd = vel_error * kp + kd*(vel_error - prev_vel_error)/TS;

    y[0] = actuator_cmd;

    prev_vel_error = vel_error;
    
}

static void mdlUpdate(real_T *x, const real_T *u, SimStruct *S, int_T tid)
{ // You can leave this empty if no updates are needed
    /* This function is called at each time step to update the state */
    /* Example: Update the first state with the first input */
    // x[0] = u[0];  // Update first state with first input

}

static void mdlDerivatives(real_T *dx, const real_T *x, const real_T *u, 
                           SimStruct *S, int_T tid)
{ // You can leave this empty if no derivatives are needed
    /* This function is called to compute the derivatives of the states */
    /* Example: Set the first derivative to zero */
    // dx[0] = 0.0;  // No change in first state

    // leave empty
}

static void mdlTerminate(SimStruct *S)
{ // You can leave this empty if no termination actions are needed
    /* This function is called at the end of the simulation */
    /* Example: Print a message */
    // mexPrintf("Simulation terminated.\n");

    // leave empty
}

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef	MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

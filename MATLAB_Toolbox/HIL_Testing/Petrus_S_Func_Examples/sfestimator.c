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
#include "esl_math.h"
#include "adcs_estimator.h"

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

double TS, orbits[7];
Vector3 sun_b, sun_o, earth_b, earth_o, eRPY, moon_b, moon_o, wbo, wo, wbi, eQ2_vec;
DCM eA_ob;
Quaternion eQ, eQ2;
ADCS_ESTIMATOR_GyroEKFParameters_t gyro_params;

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
    ssSetNumInputs(        S, 25);  /* number of inputs                      */
    ssSetNumOutputs(       S, 21);  /* number of outputs                     */
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

    // leave empty    

    // Initialize the ADCS estimator parameters
    
    ADCS_ESTIMATOR_initGyroEKF(&gyro_params); // Initialize the Gyro EKF parameters
}

static void mdlOutputs(real_T *y, const real_T *x, const real_T *u, 
                       SimStruct *S, int_T tid)
{ // This function is called at each time step to compute the outputs
    /*  u is an array of the inputs 
        y is an array of the outputs 
        
        Use your inputs as u[0]...u[i] and set your outputs as y[0]...y[i]

        Do your operations inbetween reading the inputs and setting the outputs.
    */

    orbits[0] = u[0]; orbits[1] = u[1]; orbits[2] = u[2]; orbits[3] = u[3]; // Keplerian elements (T, w0, nu, r)
    orbits[4] = u[4]; orbits[5] = u[5]; orbits[6] = u[6]; // Geocentric coordinates (lat, lon, alt)
    sun_o.x = u[7]; sun_o.y = u[8]; sun_o.z = u[9]; // sun vector - orbit referenced
    sun_b.x = u[10]; sun_b.y = u[11]; sun_b.z = u[12]; // sun vector - body referenced
    earth_o.x = u[13]; earth_o.y = u[14]; earth_o.z = u[15]; // earth vector - orbit referenced
    earth_b.x = u[16]; earth_b.y = u[17]; earth_b.z = u[18]; // earth vector - body referenced
    Vector3 wbi = {u[19], u[20], u[21]}; // gyro measurement in body frame
    Vector3 wo = {u[22], u[23], u[24]}; // orbital rate

    // mexPrintf("sun_o: %f, %f, %f", sun_b.x, sun_b.y, sun_b.z);
    
    // check if vectors are valid
    if (VECTOR3_length(sun_b) < 1e-6 || VECTOR3_length(sun_o) < 1e-6 ||
        VECTOR3_length(earth_b) < 1e-6 || VECTOR3_length(earth_o) < 1e-6) {
        mexPrintf("Invalid input vectors\n");
        return;
    } else {
        // run estimators only if inputs are valid
        // TRIAD
        eA_ob = ADCS_ESTIMATOR_updateTRIAD(sun_b, earth_b, sun_o, earth_o);
        DCM_to_euler(&eA_ob, &eRPY.x, &eRPY.y, &eRPY.z);
        eQ = QUATERNION_from_dcm(&eA_ob);

        mexPrintf("TRIAD Quaternion: %f, %f, %f, %f\n", eQ.x, eQ.y, eQ.z, eQ.w);

        // QUEST
        // double vec_b[][3] = {
        //     {sun_b.x, sun_b.y, sun_b.z},
        //     {earth_b.x, earth_b.y, earth_b.z}
        // };
        // double vec_o[][3] = {
        //     {sun_o.x, sun_o.y, sun_o.z},
        //     {earth_o.x, earth_o.y, earth_o.z}
        // };
        // Quaternion eQ2;
        // double weights[] = {1.0, 1.0};
        // // normalize weights, weights should sum to 1.0
        // double weight_sum = 0.0;
        // int weight_count = sizeof(weights) / sizeof(weights[0]);
        // for (int i = 0; i < weight_count; i++) {
        //     weight_sum += weights[i];
        // }
        // for (int i = 0; i < weight_count; i++) {
        //     weights[i] /= weight_sum;
        // } 
        // int quest = ADCS_ESTIMATOR_updateQUEST(vec_b, vec_o, weights, weight_count, &eQ2);

        // mexPrintf("QUEST result: %d, eQ2: %f, %f, %f, %f\n", quest, eQ2.x, eQ2.y, eQ2.z, eQ2.w);

        // GyroEKF
        gyro_params.state[0] = eQ.x; // Set the quaternion state from the TRIAD result
        gyro_params.state[1] = eQ.y;
        gyro_params.state[2] = eQ.z;
        gyro_params.state[3] = 0.0; // Set the gyro bias to zero for now
        gyro_params.state[4] = 0.0; // Set the gyro bias to zero for now
        gyro_params.state[5] = 0.0; // Set the gyro bias to zero for now
        gyro_params.prevQ = eQ; // Set the previous quaternion state to the TRIAD result
        ADCS_ESTIMATOR_propagateGyroEKF(&gyro_params, &wbi, &wo, &wbo);
        ADCS_ESTIMATOR_updateGyroEKF(&gyro_params, &sun_b, &sun_o);

        eQ2_vec.x = gyro_params.state[0];
        eQ2_vec.y = gyro_params.state[1];
        eQ2_vec.z = gyro_params.state[2];
        eQ2 = QUATERNION_findScalar(eQ2_vec, gyro_params.prevQ); // Find the quaternion from the state vector and previous quaternion

        // mexPrintf("GyroEKF Quaternion: %f, %f, %f, %f\n", eQ2.x, eQ2.y, eQ2.z, eQ2.w);

    }

    y[0] = eA_ob.data[0][0]; y[1] = eA_ob.data[0][1]; y[2] = eA_ob.data[0][2]; // Estimated DCM
    y[3] = eA_ob.data[1][0]; y[4] = eA_ob.data[1][1]; y[5] = eA_ob.data[1][2]; 
    y[6] = eA_ob.data[2][0]; y[7] = eA_ob.data[2][1]; y[8] = eA_ob.data[2][2]; 
    y[9] = eQ.x; y[10] = eQ.y; y[11] = eQ.z; y[12] = eQ.w; // Estimated Quaternion
    y[13] = eRPY.x; y[14] = eRPY.y; y[15] = eRPY.z; // Roll, Pitch, Yaw
    y[14] = eQ2.x; y[15] = eQ2.y; y[16] = eQ2.z; y[17] = eQ2.w; // GyroEKF Quaternion
    y[18] = wbo.x; y[19] = wbo.y; y[20] = wbo.z; // Propagated body rates
}

static void mdlUpdate(real_T *x, const real_T *u, SimStruct *S, int_T tid)
{ // You can leave this empty if no updates are needed
    /* This function is called at each time step to update the state */
    /* Example: Update the first state with the first input */
    // x[0] = u[0];  // Update first state with first input

    // leave empty
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

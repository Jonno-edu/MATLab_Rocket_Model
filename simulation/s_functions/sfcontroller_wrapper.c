#define S_FUNCTION_NAME  sfcontroller
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "mex.h"
#include <math.h>

/* Parameter indices */
enum { P_TS = 0, NUM_PARAMS };

/* DWork indices */
enum { DW_THETA_INT = 0, DW_W_ERR_INT_L = 1, DW_W_ERR_INT_Q = 2, NUM_DWORK };

/* Controller gains (outer loop on theta -> w_cmd; inner loop on w -> actuator) */
static const real_T l_kp_inner = 0.0671;
static const real_T l_ki_inner = 0.0119;
static const real_T l_kd_inner = 0.0;

static const real_T l_kp_outer = 0.0813;
static const real_T l_ki_outer = 0.0041;
static const real_T l_kd_outer = 0.0;

static const real_T Q_kp_inner = 0.9925;
static const real_T Q_ki_inner = 1.5251;
static const real_T Q_kd_inner = 0.0;

static const real_T Q_kp_outer = 0.7421;
static const real_T Q_ki_outer = 0.0156;
static const real_T Q_kd_outer = 0.0;

/* Blend reference */
static const real_T qbar_max = 78000.0 * 1.2; /* = 93600.0 */

/*========================*
 * S-function methods     *
 *========================*/

static void mdlInitializeSizes(SimStruct *S)
{
    /* Parameters */
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    /* Four scalar input ports: [Qbar, w, theta, theta_cmd] */
    if (!ssSetNumInputPorts(S, 4)) return;
    for (int_T i = 0; i < 4; ++i) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortRequiredContiguous(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 1);
    }

    /* One scalar output port: actuator command */
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);

    /* Sample times */
    ssSetNumSampleTimes(S, 1);

    /* DWork for integrators: theta_error_integral, w_cmd_error_integral_launch, w_cmd_error_integral_q */
    ssSetNumDWork(S, NUM_DWORK);
    ssSetDWorkWidth(S, DW_THETA_INT,     1);
    ssSetDWorkWidth(S, DW_W_ERR_INT_L,   1);
    ssSetDWorkWidth(S, DW_W_ERR_INT_Q,   1);
    ssSetDWorkDataType(S, DW_THETA_INT,   SS_DOUBLE);
    ssSetDWorkDataType(S, DW_W_ERR_INT_L, SS_DOUBLE);
    ssSetDWorkDataType(S, DW_W_ERR_INT_Q, SS_DOUBLE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    real_T Ts = mxGetScalar(ssGetSFcnParam(S, P_TS));
    ssSetSampleTime(S, 0, Ts);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *theta_int   = (real_T*) ssGetDWork(S, DW_THETA_INT);
    real_T *wint_launch = (real_T*) ssGetDWork(S, DW_W_ERR_INT_L);
    real_T *wint_q      = (real_T*) ssGetDWork(S, DW_W_ERR_INT_Q);

    *theta_int   = 0.0;
    *wint_launch = 0.0;
    *wint_q      = 0.0;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* Read inputs */
    const real_T Qbar      = *(const real_T*) ssGetInputPortSignal(S, 0);
    const real_T w         = *(const real_T*) ssGetInputPortSignal(S, 1);
    const real_T theta     = *(const real_T*) ssGetInputPortSignal(S, 2);
    const real_T theta_cmd = *(const real_T*) ssGetInputPortSignal(S, 3);

    /* Access DWork integrators */
    real_T *theta_int   = (real_T*) ssGetDWork(S, DW_THETA_INT);
    real_T *wint_launch = (real_T*) ssGetDWork(S, DW_W_ERR_INT_L);
    real_T *wint_q      = (real_T*) ssGetDWork(S, DW_W_ERR_INT_Q);

    const real_T Ts = ssGetSampleTime(S, 0);

    /* Outer loop: theta -> w_cmd */
    const real_T theta_error = theta_cmd - theta;
    *theta_int += theta_error * Ts;

    const real_T w_cmd_launch = l_kp_outer * theta_error + l_ki_outer * (*theta_int);
    const real_T w_cmd_q      = Q_kp_outer * theta_error + Q_ki_outer * (*theta_int);

    /* Inner loop: w_cmd -> actuator (PI) */
    const real_T w_err_launch = w_cmd_launch - w;
    const real_T w_err_q      = w_cmd_q - w;

    *wint_launch += w_err_launch * Ts;
    *wint_q      += w_err_q * Ts;

    const real_T act_cmd_launch = l_kp_inner * w_err_launch + l_ki_inner * (*wint_launch);
    const real_T act_cmd_q      = Q_kp_inner * w_err_q      + Q_ki_inner * (*wint_q);

    /* Blend by dynamic pressure ratio, clamped to [0,1] */
    real_T w_blend = Qbar / qbar_max;
    if (w_blend < 0.0) w_blend = 0.0;
    else if (w_blend > 1.0) w_blend = 1.0;

    /* Output */
    real_T *y = (real_T*) ssGetOutputPortSignal(S, 0);
    y[0] = (1.0 - w_blend) * act_cmd_launch + w_blend * act_cmd_q;
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
    /* All integrators updated in mdlOutputs for this discrete controller */
    UNUSED_ARG(S);
    UNUSED_ARG(tid);
}

static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S);
}

/*=============================*
 * Required S-function trailer *
 *=============================*/
#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

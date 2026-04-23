/*
 * truesigma.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "truesigma".
 *
 * Model version              : 1.55
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C source code generated on : Thu Apr 23 15:14:37 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef truesigma_h_
#define truesigma_h_
#ifndef truesigma_COMMON_INCLUDES_
#define truesigma_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* truesigma_COMMON_INCLUDES_ */

#include "truesigma_types.h"
#include <string.h>
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Saturation;                   /* '<S1>/Saturation' */
  real_T Saturation1;                  /* '<S1>/Saturation1' */
  real_T Saturation2;                  /* '<S1>/Saturation2' */
  real_T CustomVariableMass6DOFEulerAngl[3];
                         /* '<Root>/Custom Variable Mass 6DOF (Euler Angles)' */
  real_T CustomVariableMass6DOFEulerAn_i[3];
                         /* '<Root>/Custom Variable Mass 6DOF (Euler Angles)' */
  real_T CustomVariableMass6DOFEulerAn_o[3];
                         /* '<Root>/Custom Variable Mass 6DOF (Euler Angles)' */
  real_T RateLimiter;                  /* '<S1>/Rate Limiter' */
  real_T RateLimiter1;                 /* '<S1>/Rate Limiter1' */
  real_T RateLimiter2;                 /* '<S1>/Rate Limiter2' */
  real_T Thrust;                       /* '<Root>/Thrust' */
  real_T forcesum[3];                  /* '<Root>/force sum' */
  real_T mass;                         /* '<Root>/propulsion' */
  real_T euler_cmd[3];                 /* '<Root>/guidance' */
  real_T M_aero_body[3];               /* '<Root>/areodynamics' */
} B_truesigma_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T PrevY;                        /* '<S1>/Rate Limiter' */
  real_T LastMajorTime;                /* '<S1>/Rate Limiter' */
  real_T PrevY_i;                      /* '<S1>/Rate Limiter1' */
  real_T LastMajorTime_h;              /* '<S1>/Rate Limiter1' */
  real_T PrevY_f;                      /* '<S1>/Rate Limiter2' */
  real_T LastMajorTime_m;              /* '<S1>/Rate Limiter2' */
  boolean_T PrevLimited;               /* '<S1>/Rate Limiter' */
  boolean_T PrevLimited_e;             /* '<S1>/Rate Limiter1' */
  boolean_T PrevLimited_n;             /* '<S1>/Rate Limiter2' */
} DW_truesigma_T;

/* Continuous states (default storage) */
typedef struct {
  real_T TransferFcn_CSTATE;           /* '<S1>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S1>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE;          /* '<S1>/Transfer Fcn2' */
  real_T CustomVariableMass6DOFEulerAngl[12];
                         /* '<Root>/Custom Variable Mass 6DOF (Euler Angles)' */
} X_truesigma_T;

/* Periodic continuous state vector (global) */
typedef int_T PeriodicIndX_truesigma_T[3];
typedef real_T PeriodicRngX_truesigma_T[6];

/* State derivatives (default storage) */
typedef struct {
  real_T TransferFcn_CSTATE;           /* '<S1>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S1>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE;          /* '<S1>/Transfer Fcn2' */
  real_T CustomVariableMass6DOFEulerAngl[12];
                         /* '<Root>/Custom Variable Mass 6DOF (Euler Angles)' */
} XDot_truesigma_T;

/* State disabled  */
typedef struct {
  boolean_T TransferFcn_CSTATE;        /* '<S1>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE;       /* '<S1>/Transfer Fcn1' */
  boolean_T TransferFcn2_CSTATE;       /* '<S1>/Transfer Fcn2' */
  boolean_T CustomVariableMass6DOFEulerAngl[12];
                         /* '<Root>/Custom Variable Mass 6DOF (Euler Angles)' */
} XDis_truesigma_T;

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
} ODE4_IntgData;

#endif

/* Parameters (default storage) */
struct P_truesigma_T_ {
  real_T TransferFcn_A;                /* Computed Parameter: TransferFcn_A
                                        * Referenced by: '<S1>/Transfer Fcn'
                                        */
  real_T TransferFcn_C;                /* Computed Parameter: TransferFcn_C
                                        * Referenced by: '<S1>/Transfer Fcn'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 0.26
                                        * Referenced by: '<S1>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: -0.26
                                        * Referenced by: '<S1>/Saturation'
                                        */
  real_T TransferFcn1_A;               /* Computed Parameter: TransferFcn1_A
                                        * Referenced by: '<S1>/Transfer Fcn1'
                                        */
  real_T TransferFcn1_C;               /* Computed Parameter: TransferFcn1_C
                                        * Referenced by: '<S1>/Transfer Fcn1'
                                        */
  real_T Saturation1_UpperSat;         /* Expression: 0.26
                                        * Referenced by: '<S1>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: -0.26
                                        * Referenced by: '<S1>/Saturation1'
                                        */
  real_T TransferFcn2_A;               /* Computed Parameter: TransferFcn2_A
                                        * Referenced by: '<S1>/Transfer Fcn2'
                                        */
  real_T TransferFcn2_C;               /* Computed Parameter: TransferFcn2_C
                                        * Referenced by: '<S1>/Transfer Fcn2'
                                        */
  real_T Saturation2_UpperSat;         /* Expression: 0.26
                                        * Referenced by: '<S1>/Saturation2'
                                        */
  real_T Saturation2_LowerSat;         /* Expression: -0.26
                                        * Referenced by: '<S1>/Saturation2'
                                        */
  real_T CustomVariableMass6DOFEulerAngl[3];/* Expression: [0 0 0]
                                             * Referenced by: '<Root>/Custom Variable Mass 6DOF (Euler Angles)'
                                             */
  real_T CustomVariableMass6DOFEulerAn_o[3];/* Expression: [0 0 0]
                                             * Referenced by: '<Root>/Custom Variable Mass 6DOF (Euler Angles)'
                                             */
  real_T CustomVariableMass6DOFEulerAn_a[3];/* Expression: [0 pi/2 0]
                                             * Referenced by: '<Root>/Custom Variable Mass 6DOF (Euler Angles)'
                                             */
  real_T CustomVariableMass6DOFEulerAn_e[3];/* Expression: [0 0 0]
                                             * Referenced by: '<Root>/Custom Variable Mass 6DOF (Euler Angles)'
                                             */
  real_T waypoint_Value[3];            /* Expression: [150; 0; -300]
                                        * Referenced by: '<Root>/waypoint'
                                        */
  real_T Constant_Value[9];            /* Expression: zeros(3)
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T Step1_Time;                   /* Expression: 2
                                        * Referenced by: '<Root>/Step1'
                                        */
  real_T Step1_Y0;                     /* Expression: 0
                                        * Referenced by: '<Root>/Step1'
                                        */
  real_T Step1_YFinal;                 /* Expression: -37.5
                                        * Referenced by: '<Root>/Step1'
                                        */
  real_T RateLimiter_RisingLim;        /* Expression: 10
                                        * Referenced by: '<S1>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim;       /* Expression: -10
                                        * Referenced by: '<S1>/Rate Limiter'
                                        */
  real_T RateLimiter1_RisingLim;       /* Expression: 10
                                        * Referenced by: '<S1>/Rate Limiter1'
                                        */
  real_T RateLimiter1_FallingLim;      /* Expression: -10
                                        * Referenced by: '<S1>/Rate Limiter1'
                                        */
  real_T RateLimiter2_RisingLim;       /* Expression: 10
                                        * Referenced by: '<S1>/Rate Limiter2'
                                        */
  real_T RateLimiter2_FallingLim;      /* Expression: -10
                                        * Referenced by: '<S1>/Rate Limiter2'
                                        */
  real_T Thrust_Time;                  /* Expression: 0
                                        * Referenced by: '<Root>/Thrust'
                                        */
  real_T Thrust_Y0;                    /* Expression: 0
                                        * Referenced by: '<Root>/Thrust'
                                        */
  real_T Thrust_YFinal;                /* Expression: 37.5
                                        * Referenced by: '<Root>/Thrust'
                                        */
  real_T inertia_Value[9];           /* Expression: diag([0.0002, 0.01, 0.0146])
                                      * Referenced by: '<Root>/inertia'
                                      */
};

/* Real-time Model Data Structure */
struct tag_RTM_truesigma_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;
  X_truesigma_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_truesigma_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[15];
  real_T odeF[4][15];
  ODE4_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tStart;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
extern P_truesigma_T truesigma_P;

/* Block signals (default storage) */
extern B_truesigma_T truesigma_B;

/* Continuous states (default storage) */
extern X_truesigma_T truesigma_X;

/* Disabled states (default storage) */
extern XDis_truesigma_T truesigma_XDis;

/* Block states (default storage) */
extern DW_truesigma_T truesigma_DW;

/* Model entry point functions */
extern void truesigma_initialize(void);
extern void truesigma_step(void);
extern void truesigma_terminate(void);

/* Real-time Model object */
extern RT_MODEL_truesigma_T *const truesigma_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'truesigma'
 * '<S1>'   : 'truesigma/Subsystem'
 * '<S2>'   : 'truesigma/areodynamics'
 * '<S3>'   : 'truesigma/attitude_ctrl'
 * '<S4>'   : 'truesigma/environment'
 * '<S5>'   : 'truesigma/gravity-body'
 * '<S6>'   : 'truesigma/guidance'
 * '<S7>'   : 'truesigma/propulsion'
 */
#endif                                 /* truesigma_h_ */

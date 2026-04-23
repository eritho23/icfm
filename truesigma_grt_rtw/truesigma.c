/*
 * truesigma.c
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

#include "truesigma.h"
#include <emmintrin.h>
#include <math.h>
#include "truesigma_private.h"
#include "rtwtypes.h"
#include <string.h>
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include <float.h>

/* Block signals (default storage) */
B_truesigma_T truesigma_B;

/* Continuous states */
X_truesigma_T truesigma_X;

/* Disabled State Vector */
XDis_truesigma_T truesigma_XDis;

/* Block states (default storage) */
DW_truesigma_T truesigma_DW;

/* Periodic continuous states */
PeriodicIndX_truesigma_T truesigma_PeriodicIndX;
PeriodicRngX_truesigma_T truesigma_PeriodicRngX;

/* Real-time model */
static RT_MODEL_truesigma_T truesigma_M_;
RT_MODEL_truesigma_T *const truesigma_M = &truesigma_M_;

/* Projection for root system: '<Root>' */
void truesigma_projection(void)
{
  real_T newangle;

  /* Projection for EOM6DOFBodyEuler: '<Root>/Custom Variable Mass 6DOF (Euler Angles)' */
  newangle = truesigma_X.CustomVariableMass6DOFEulerAngl[9];
  if (fabs(truesigma_X.CustomVariableMass6DOFEulerAngl[9]) > 3.1415926535897931)
  {
    newangle = rt_remd_snf(truesigma_X.CustomVariableMass6DOFEulerAngl[9],
      6.2831853071795862);
    newangle -= trunc(newangle / 3.1415926535897931) * 6.2831853071795862;
  }

  truesigma_X.CustomVariableMass6DOFEulerAngl[9] = newangle;
  newangle = truesigma_X.CustomVariableMass6DOFEulerAngl[10];
  if (fabs(truesigma_X.CustomVariableMass6DOFEulerAngl[10]) > 3.1415926535897931)
  {
    newangle = rt_remd_snf(truesigma_X.CustomVariableMass6DOFEulerAngl[10],
      6.2831853071795862);
    newangle -= trunc(newangle / 3.1415926535897931) * 6.2831853071795862;
  }

  truesigma_X.CustomVariableMass6DOFEulerAngl[10] = newangle;
  newangle = truesigma_X.CustomVariableMass6DOFEulerAngl[11];
  if (fabs(truesigma_X.CustomVariableMass6DOFEulerAngl[11]) > 3.1415926535897931)
  {
    newangle = rt_remd_snf(truesigma_X.CustomVariableMass6DOFEulerAngl[11],
      6.2831853071795862);
    newangle -= trunc(newangle / 3.1415926535897931) * 6.2831853071795862;
  }

  truesigma_X.CustomVariableMass6DOFEulerAngl[11] = newangle;

  /* End of Projection for EOM6DOFBodyEuler: '<Root>/Custom Variable Mass 6DOF (Euler Angles)' */
}

/* State reduction function */
void local_stateReduction(real_T* x, int_T* p, int_T n, real_T* r)
{
  int_T i, j;
  for (i = 0, j = 0; i < n; ++i, ++j) {
    int_T k = p[i];
    real_T lb = r[j++];
    real_T xk = x[k]-lb;
    real_T rk = r[j]-lb;
    int_T q = (int_T) floor(xk/rk);
    if (q) {
      x[k] = xk-q*rk+lb;
    }
  }
}

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 15;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  truesigma_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  truesigma_step();
  truesigma_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  truesigma_step();
  truesigma_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  truesigma_step();
  truesigma_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  truesigma_step();
  truesigma_projection();
  local_stateReduction(rtsiGetContStates(si), rtsiGetPeriodicContStateIndices(si),
                       3,
                       rtsiGetPeriodicContStateRanges(si));
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

real_T rt_remd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = (rtNaN);
  } else if (rtIsInf(u1)) {
    y = u0;
  } else if ((u1 != 0.0) && (u1 != trunc(u1))) {
    real_T q;
    q = fabs(u0 / u1);
    if (!(fabs(q - floor(q + 0.5)) > DBL_EPSILON * q)) {
      y = 0.0 * u0;
    } else {
      y = fmod(u0, u1);
    }
  } else {
    y = fmod(u0, u1);
  }

  return y;
}

/* Model step function */
void truesigma_step(void)
{
  if (rtmIsMajorTimeStep(truesigma_M)) {
    /* set solver stop time */
    if (!(truesigma_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&truesigma_M->solverInfo,
                            ((truesigma_M->Timing.clockTickH0 + 1) *
        truesigma_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&truesigma_M->solverInfo,
                            ((truesigma_M->Timing.clockTick0 + 1) *
        truesigma_M->Timing.stepSize0 + truesigma_M->Timing.clockTickH0 *
        truesigma_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(truesigma_M)) {
    truesigma_M->Timing.t[0] = rtsiGetT(&truesigma_M->solverInfo);
  }

  {
    __m128d tmp_1;
    __m128d tmp_2;
    __m128d tmp_3;
    real_T cth_0[9];
    real_T tmp_0[2];
    real_T V_safe;
    real_T alpha;
    real_T beta;
    real_T cpsi;
    real_T cth;
    real_T cth_tmp;
    real_T delta_idx_2;
    real_T horiz_range;
    real_T rtb_Step1;
    real_T spsi;
    real_T sth;
    int32_T i;
    boolean_T limitedCache;
    boolean_T tmp;
    static const real_T b[3] = { 0.0, 0.0, 9.81 };

    /* TransferFcn: '<S1>/Transfer Fcn' */
    rtb_Step1 = truesigma_P.TransferFcn_C * truesigma_X.TransferFcn_CSTATE;

    /* Saturate: '<S1>/Saturation' */
    if (rtb_Step1 > truesigma_P.Saturation_UpperSat) {
      /* Saturate: '<S1>/Saturation' */
      truesigma_B.Saturation = truesigma_P.Saturation_UpperSat;
    } else if (rtb_Step1 < truesigma_P.Saturation_LowerSat) {
      /* Saturate: '<S1>/Saturation' */
      truesigma_B.Saturation = truesigma_P.Saturation_LowerSat;
    } else {
      /* Saturate: '<S1>/Saturation' */
      truesigma_B.Saturation = rtb_Step1;
    }

    /* End of Saturate: '<S1>/Saturation' */

    /* TransferFcn: '<S1>/Transfer Fcn1' */
    rtb_Step1 = truesigma_P.TransferFcn1_C * truesigma_X.TransferFcn1_CSTATE;

    /* Saturate: '<S1>/Saturation1' */
    if (rtb_Step1 > truesigma_P.Saturation1_UpperSat) {
      /* Saturate: '<S1>/Saturation1' */
      truesigma_B.Saturation1 = truesigma_P.Saturation1_UpperSat;
    } else if (rtb_Step1 < truesigma_P.Saturation1_LowerSat) {
      /* Saturate: '<S1>/Saturation1' */
      truesigma_B.Saturation1 = truesigma_P.Saturation1_LowerSat;
    } else {
      /* Saturate: '<S1>/Saturation1' */
      truesigma_B.Saturation1 = rtb_Step1;
    }

    /* End of Saturate: '<S1>/Saturation1' */

    /* TransferFcn: '<S1>/Transfer Fcn2' */
    rtb_Step1 = truesigma_P.TransferFcn2_C * truesigma_X.TransferFcn2_CSTATE;

    /* Saturate: '<S1>/Saturation2' */
    if (rtb_Step1 > truesigma_P.Saturation2_UpperSat) {
      /* Saturate: '<S1>/Saturation2' */
      truesigma_B.Saturation2 = truesigma_P.Saturation2_UpperSat;
    } else if (rtb_Step1 < truesigma_P.Saturation2_LowerSat) {
      /* Saturate: '<S1>/Saturation2' */
      truesigma_B.Saturation2 = truesigma_P.Saturation2_LowerSat;
    } else {
      /* Saturate: '<S1>/Saturation2' */
      truesigma_B.Saturation2 = rtb_Step1;
    }

    /* End of Saturate: '<S1>/Saturation2' */
    tmp = rtmIsMajorTimeStep(truesigma_M);
    if (tmp) {
    }

    /* EOM6DOFBodyEuler: '<Root>/Custom Variable Mass 6DOF (Euler Angles)' */
    truesigma_B.CustomVariableMass6DOFEulerAn_o[0] =
      truesigma_X.CustomVariableMass6DOFEulerAngl[0];
    truesigma_B.CustomVariableMass6DOFEulerAn_o[1] =
      truesigma_X.CustomVariableMass6DOFEulerAngl[1];
    truesigma_B.CustomVariableMass6DOFEulerAn_o[2] =
      truesigma_X.CustomVariableMass6DOFEulerAngl[2];
    truesigma_B.CustomVariableMass6DOFEulerAngl[0] =
      truesigma_X.CustomVariableMass6DOFEulerAngl[3];
    truesigma_B.CustomVariableMass6DOFEulerAngl[1] =
      truesigma_X.CustomVariableMass6DOFEulerAngl[4];
    truesigma_B.CustomVariableMass6DOFEulerAngl[2] =
      truesigma_X.CustomVariableMass6DOFEulerAngl[5];
    truesigma_B.CustomVariableMass6DOFEulerAn_i[0] =
      truesigma_X.CustomVariableMass6DOFEulerAngl[9];
    truesigma_B.CustomVariableMass6DOFEulerAn_i[1] =
      truesigma_X.CustomVariableMass6DOFEulerAngl[10];
    truesigma_B.CustomVariableMass6DOFEulerAn_i[2] =
      truesigma_X.CustomVariableMass6DOFEulerAngl[11];
    if (tmp) {
    }

    /* Constant: '<Root>/waypoint' incorporates:
     *  MATLAB Function: '<Root>/guidance'
     */
    tmp_1 = _mm_sub_pd(_mm_loadu_pd(&truesigma_P.waypoint_Value[0]),
                       _mm_loadu_pd
                       (&truesigma_B.CustomVariableMass6DOFEulerAngl[0]));
    _mm_storeu_pd(&tmp_0[0], tmp_1);

    /* MATLAB Function: '<Root>/guidance' incorporates:
     *  Constant: '<Root>/waypoint'
     */
    delta_idx_2 = truesigma_P.waypoint_Value[2] -
      truesigma_B.CustomVariableMass6DOFEulerAngl[2];
    alpha = tmp_0[0] * tmp_0[0] + tmp_0[1] * tmp_0[1];
    horiz_range = sqrt(alpha);
    if (sqrt(delta_idx_2 * delta_idx_2 + alpha) < 30.0) {
      truesigma_B.euler_cmd[1] = 1.5707963267948966;
      truesigma_B.euler_cmd[2] = 0.0;
    } else {
      truesigma_B.euler_cmd[1] = rt_atan2d_snf(-delta_idx_2, fmax(horiz_range,
        0.1));
      if (horiz_range < 1.0) {
        truesigma_B.euler_cmd[2] = 0.0;
      } else {
        truesigma_B.euler_cmd[2] = rt_atan2d_snf(tmp_0[1], tmp_0[0]);
      }
    }

    truesigma_B.euler_cmd[0] = 0.0;
    if (tmp) {
    }

    /* Clock: '<Root>/Clock' */
    rtb_Step1 = truesigma_M->Timing.t[0];

    /* MATLAB Function: '<Root>/attitude_ctrl' incorporates:
     *  EOM6DOFBodyEuler: '<Root>/Custom Variable Mass 6DOF (Euler Angles)'
     */
    tmp_1 = _mm_sub_pd(_mm_sub_pd(_mm_loadu_pd(&truesigma_B.euler_cmd[1]),
      _mm_loadu_pd(&truesigma_B.CustomVariableMass6DOFEulerAn_i[1])), _mm_mul_pd
                       (_mm_set1_pd(0.3), _mm_loadu_pd
                        (&truesigma_X.CustomVariableMass6DOFEulerAngl[7])));
    _mm_storeu_pd(&tmp_0[0], tmp_1);

    /* MATLAB Function: '<Root>/attitude_ctrl' incorporates:
     *  EOM6DOFBodyEuler: '<Root>/Custom Variable Mass 6DOF (Euler Angles)'
     */
    delta_idx_2 = (truesigma_B.euler_cmd[0] -
                   truesigma_B.CustomVariableMass6DOFEulerAn_i[0]) * 0.2 - 0.05 *
      truesigma_X.CustomVariableMass6DOFEulerAngl[6];

    /* RateLimiter: '<S1>/Rate Limiter' */
    if (truesigma_DW.LastMajorTime == (rtInf)) {
      /* RateLimiter: '<S1>/Rate Limiter' */
      truesigma_B.RateLimiter = tmp_0[0];
    } else {
      alpha = truesigma_M->Timing.t[0];
      horiz_range = alpha - truesigma_DW.LastMajorTime;
      if (truesigma_DW.LastMajorTime == alpha) {
        if (truesigma_DW.PrevLimited) {
          /* RateLimiter: '<S1>/Rate Limiter' */
          truesigma_B.RateLimiter = truesigma_DW.PrevY;
        } else {
          /* RateLimiter: '<S1>/Rate Limiter' */
          truesigma_B.RateLimiter = tmp_0[0];
        }
      } else {
        beta = horiz_range * truesigma_P.RateLimiter_RisingLim;
        alpha = tmp_0[0] - truesigma_DW.PrevY;
        if (alpha > beta) {
          /* RateLimiter: '<S1>/Rate Limiter' */
          truesigma_B.RateLimiter = truesigma_DW.PrevY + beta;
          limitedCache = true;
        } else {
          horiz_range *= truesigma_P.RateLimiter_FallingLim;
          if (alpha < horiz_range) {
            /* RateLimiter: '<S1>/Rate Limiter' */
            truesigma_B.RateLimiter = truesigma_DW.PrevY + horiz_range;
            limitedCache = true;
          } else {
            /* RateLimiter: '<S1>/Rate Limiter' */
            truesigma_B.RateLimiter = tmp_0[0];
            limitedCache = false;
          }
        }

        if (rtsiIsModeUpdateTimeStep(&truesigma_M->solverInfo)) {
          truesigma_DW.PrevLimited = limitedCache;
        }
      }
    }

    /* End of RateLimiter: '<S1>/Rate Limiter' */

    /* RateLimiter: '<S1>/Rate Limiter1' */
    if (truesigma_DW.LastMajorTime_h == (rtInf)) {
      /* RateLimiter: '<S1>/Rate Limiter1' */
      truesigma_B.RateLimiter1 = tmp_0[1];
    } else {
      alpha = truesigma_M->Timing.t[0];
      horiz_range = alpha - truesigma_DW.LastMajorTime_h;
      if (truesigma_DW.LastMajorTime_h == alpha) {
        if (truesigma_DW.PrevLimited_e) {
          /* RateLimiter: '<S1>/Rate Limiter1' */
          truesigma_B.RateLimiter1 = truesigma_DW.PrevY_i;
        } else {
          /* RateLimiter: '<S1>/Rate Limiter1' */
          truesigma_B.RateLimiter1 = tmp_0[1];
        }
      } else {
        beta = horiz_range * truesigma_P.RateLimiter1_RisingLim;
        alpha = tmp_0[1] - truesigma_DW.PrevY_i;
        if (alpha > beta) {
          /* RateLimiter: '<S1>/Rate Limiter1' */
          truesigma_B.RateLimiter1 = truesigma_DW.PrevY_i + beta;
          limitedCache = true;
        } else {
          horiz_range *= truesigma_P.RateLimiter1_FallingLim;
          if (alpha < horiz_range) {
            /* RateLimiter: '<S1>/Rate Limiter1' */
            truesigma_B.RateLimiter1 = truesigma_DW.PrevY_i + horiz_range;
            limitedCache = true;
          } else {
            /* RateLimiter: '<S1>/Rate Limiter1' */
            truesigma_B.RateLimiter1 = tmp_0[1];
            limitedCache = false;
          }
        }

        if (rtsiIsModeUpdateTimeStep(&truesigma_M->solverInfo)) {
          truesigma_DW.PrevLimited_e = limitedCache;
        }
      }
    }

    /* End of RateLimiter: '<S1>/Rate Limiter1' */

    /* RateLimiter: '<S1>/Rate Limiter2' */
    if (truesigma_DW.LastMajorTime_m == (rtInf)) {
      /* RateLimiter: '<S1>/Rate Limiter2' */
      truesigma_B.RateLimiter2 = delta_idx_2;
    } else {
      alpha = truesigma_M->Timing.t[0];
      horiz_range = alpha - truesigma_DW.LastMajorTime_m;
      if (truesigma_DW.LastMajorTime_m == alpha) {
        if (truesigma_DW.PrevLimited_n) {
          /* RateLimiter: '<S1>/Rate Limiter2' */
          truesigma_B.RateLimiter2 = truesigma_DW.PrevY_f;
        } else {
          /* RateLimiter: '<S1>/Rate Limiter2' */
          truesigma_B.RateLimiter2 = delta_idx_2;
        }
      } else {
        beta = horiz_range * truesigma_P.RateLimiter2_RisingLim;
        alpha = delta_idx_2 - truesigma_DW.PrevY_f;
        if (alpha > beta) {
          /* RateLimiter: '<S1>/Rate Limiter2' */
          truesigma_B.RateLimiter2 = truesigma_DW.PrevY_f + beta;
          limitedCache = true;
        } else {
          horiz_range *= truesigma_P.RateLimiter2_FallingLim;
          if (alpha < horiz_range) {
            /* RateLimiter: '<S1>/Rate Limiter2' */
            truesigma_B.RateLimiter2 = truesigma_DW.PrevY_f + horiz_range;
            limitedCache = true;
          } else {
            /* RateLimiter: '<S1>/Rate Limiter2' */
            truesigma_B.RateLimiter2 = delta_idx_2;
            limitedCache = false;
          }
        }

        if (rtsiIsModeUpdateTimeStep(&truesigma_M->solverInfo)) {
          truesigma_DW.PrevLimited_n = limitedCache;
        }
      }
    }

    /* End of RateLimiter: '<S1>/Rate Limiter2' */
    if (tmp) {
      /* Step: '<Root>/Thrust' */
      if ((((truesigma_M->Timing.clockTick1+truesigma_M->Timing.clockTickH1*
             4294967296.0)) * 1.0) < truesigma_P.Thrust_Time) {
        /* Step: '<Root>/Thrust' */
        truesigma_B.Thrust = truesigma_P.Thrust_Y0;
      } else {
        /* Step: '<Root>/Thrust' */
        truesigma_B.Thrust = truesigma_P.Thrust_YFinal;
      }

      /* End of Step: '<Root>/Thrust' */
    }

    /* MATLAB Function: '<Root>/areodynamics' incorporates:
     *  EOM6DOFBodyEuler: '<Root>/Custom Variable Mass 6DOF (Euler Angles)'
     *  SignalConversion generated from: '<S2>/ SFunction '
     */
    horiz_range = sqrt((truesigma_B.CustomVariableMass6DOFEulerAn_o[0] *
                        truesigma_B.CustomVariableMass6DOFEulerAn_o[0] +
                        truesigma_B.CustomVariableMass6DOFEulerAn_o[1] *
                        truesigma_B.CustomVariableMass6DOFEulerAn_o[1]) +
                       truesigma_B.CustomVariableMass6DOFEulerAn_o[2] *
                       truesigma_B.CustomVariableMass6DOFEulerAn_o[2]);
    V_safe = fmax(horiz_range, 0.1);
    horiz_range = horiz_range * horiz_range * 0.6125;
    alpha = rt_atan2d_snf(truesigma_B.CustomVariableMass6DOFEulerAn_o[2], V_safe);
    beta = rt_atan2d_snf(truesigma_B.CustomVariableMass6DOFEulerAn_o[1], V_safe);
    delta_idx_2 = horiz_range * 0.00126;
    truesigma_B.M_aero_body[0] = delta_idx_2 * 0.04 * 2.0 *
      truesigma_B.Saturation2;
    tmp_1 = _mm_set1_pd(0.00126);
    tmp_2 = _mm_mul_pd(_mm_set1_pd(horiz_range), tmp_1);
    tmp_3 = _mm_set1_pd(2.0);
    tmp_1 = _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_mul_pd(_mm_mul_pd(_mm_mul_pd
      (_mm_set1_pd(-horiz_range), tmp_1), _mm_set1_pd(0.1)), _mm_set1_pd(10.0)),
      _mm_set_pd(beta, alpha)), _mm_mul_pd(_mm_mul_pd(_mm_mul_pd(tmp_2,
      _mm_set1_pd(0.2)), tmp_3), _mm_set_pd(truesigma_B.Saturation1,
      truesigma_B.Saturation))), _mm_div_pd(_mm_mul_pd(_mm_mul_pd(_mm_mul_pd
      (tmp_2, _mm_set1_pd(0.0016)), _mm_set1_pd(-5.0)), _mm_loadu_pd
      (&truesigma_X.CustomVariableMass6DOFEulerAngl[7])), _mm_mul_pd(tmp_3,
      _mm_set1_pd(V_safe))));
    _mm_storeu_pd(&truesigma_B.M_aero_body[1], tmp_1);

    /* MATLAB Function: '<Root>/propulsion' */
    if (rtb_Step1 < 2.0) {
      truesigma_B.mass = 0.7 - 0.049999999999999933 * rtb_Step1 / 2.0;
    } else {
      truesigma_B.mass = 0.65;
    }

    /* MATLAB Function: '<Root>/gravity-body' */
    rtb_Step1 = cos(truesigma_B.CustomVariableMass6DOFEulerAn_i[0]);
    V_safe = sin(truesigma_B.CustomVariableMass6DOFEulerAn_i[0]);
    cth = cos(truesigma_B.CustomVariableMass6DOFEulerAn_i[1]);
    sth = sin(truesigma_B.CustomVariableMass6DOFEulerAn_i[1]);
    cpsi = cos(truesigma_B.CustomVariableMass6DOFEulerAn_i[2]);
    spsi = sin(truesigma_B.CustomVariableMass6DOFEulerAn_i[2]);
    cth_0[0] = cth * cpsi;
    cth_0[3] = cth * spsi;
    cth_0[6] = -sth;
    cth_tmp = V_safe * sth;
    cth_0[1] = cth_tmp * cpsi - rtb_Step1 * spsi;
    cth_0[4] = cth_tmp * spsi + rtb_Step1 * cpsi;
    cth_0[7] = V_safe * cth;
    cth_tmp = rtb_Step1 * sth;
    cth_0[2] = cth_tmp * cpsi + V_safe * spsi;
    cth_0[5] = cth_tmp * spsi - V_safe * cpsi;
    cth_0[8] = rtb_Step1 * cth;
    cth = 0.0;
    rtb_Step1 = 0.0;
    V_safe = 0.0;
    for (i = 0; i < 3; i++) {
      tmp_1 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&cth_0[3 * i]), _mm_set1_pd(b[i])),
                         _mm_set_pd(rtb_Step1, cth));
      _mm_storeu_pd(&tmp_0[0], tmp_1);
      cth = tmp_0[0];
      rtb_Step1 = tmp_0[1];
      V_safe += cth_0[3 * i + 2] * b[i];
    }

    /* MATLAB Function: '<Root>/propulsion' incorporates:
     *  Step: '<Root>/Step1'
     */
    if (truesigma_M->Timing.t[0] < truesigma_P.Step1_Time) {
      sth = truesigma_P.Step1_Y0;
    } else {
      sth = truesigma_P.Step1_YFinal;
    }

    /* MATLAB Function: '<Root>/areodynamics' */
    if (truesigma_B.CustomVariableMass6DOFEulerAn_o[0] > 0.0) {
      delta_idx_2 = -horiz_range * 0.00126 * 0.6;
    } else {
      delta_idx_2 *= 0.6;
    }

    /* Sum: '<Root>/force sum' incorporates:
     *  MATLAB Function: '<Root>/areodynamics'
     *  MATLAB Function: '<Root>/gravity-body'
     *  MATLAB Function: '<Root>/propulsion'
     *  Step: '<Root>/Step1'
     *  Sum: '<Root>/Sum'
     */
    truesigma_B.forcesum[0] = ((truesigma_B.Thrust + sth) + delta_idx_2) +
      truesigma_B.mass * cth;

    /* MATLAB Function: '<Root>/areodynamics' */
    horiz_range = -horiz_range * 0.00126;

    /* Sum: '<Root>/force sum' incorporates:
     *  MATLAB Function: '<Root>/areodynamics'
     *  MATLAB Function: '<Root>/gravity-body'
     *  SignalConversion generated from: '<S2>/ SFunction '
     */
    truesigma_B.forcesum[1] = (10.0 * beta + 2.0 * truesigma_B.Saturation1) *
      horiz_range + truesigma_B.mass * rtb_Step1;
    truesigma_B.forcesum[2] = (10.0 * alpha + 2.0 * truesigma_B.Saturation) *
      horiz_range + truesigma_B.mass * V_safe;
  }

  if (rtmIsMajorTimeStep(truesigma_M)) {
    /* Matfile logging */
    rt_UpdateTXYLogVars(truesigma_M->rtwLogInfo, (truesigma_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(truesigma_M)) {
    real_T LastMajorTime_tmp;

    /* Update for RateLimiter: '<S1>/Rate Limiter' incorporates:
     *  RateLimiter: '<S1>/Rate Limiter1'
     *  RateLimiter: '<S1>/Rate Limiter2'
     */
    truesigma_DW.PrevY = truesigma_B.RateLimiter;
    LastMajorTime_tmp = truesigma_M->Timing.t[0];
    truesigma_DW.LastMajorTime = LastMajorTime_tmp;

    /* Update for RateLimiter: '<S1>/Rate Limiter1' */
    truesigma_DW.PrevY_i = truesigma_B.RateLimiter1;
    truesigma_DW.LastMajorTime_h = LastMajorTime_tmp;

    /* Update for RateLimiter: '<S1>/Rate Limiter2' */
    truesigma_DW.PrevY_f = truesigma_B.RateLimiter2;
    truesigma_DW.LastMajorTime_m = LastMajorTime_tmp;
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(truesigma_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(truesigma_M)!=-1) &&
          !((rtmGetTFinal(truesigma_M)-(((truesigma_M->Timing.clockTick1+
               truesigma_M->Timing.clockTickH1* 4294967296.0)) * 1.0)) >
            (((truesigma_M->Timing.clockTick1+truesigma_M->Timing.clockTickH1*
               4294967296.0)) * 1.0) * (DBL_EPSILON))) {
        rtmSetErrorStatus(truesigma_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&truesigma_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++truesigma_M->Timing.clockTick0)) {
      ++truesigma_M->Timing.clockTickH0;
    }

    truesigma_M->Timing.t[0] = rtsiGetSolverStopTime(&truesigma_M->solverInfo);

    {
      /* Update absolute timer for sample time: [1.0s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 1.0, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      truesigma_M->Timing.clockTick1++;
      if (!truesigma_M->Timing.clockTick1) {
        truesigma_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void truesigma_derivatives(void)
{
  __m128d tmp;
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  XDot_truesigma_T *_rtXdot;
  real_T tmp_0[2];
  real_T pData_idx_0;
  real_T pData_idx_1;
  real_T pData_idx_1_tmp;
  real_T pData_idx_1_tmp_0;
  real_T pData_idx_2_tmp;
  real_T pData_idx_2_tmp_0;
  real_T pData_idx_4;
  real_T pData_idx_6;
  real_T pData_idx_6_tmp;
  real_T pData_idx_7;
  real_T pData_idx_7_tmp;
  real_T pData_tmp;
  real_T pData_tmp_0;
  real_T pData_tmp_1;
  real_T pData_tmp_2;
  _rtXdot = ((XDot_truesigma_T *) truesigma_M->derivs);

  /* Derivatives for TransferFcn: '<S1>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE = truesigma_P.TransferFcn_A *
    truesigma_X.TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE += truesigma_B.RateLimiter;

  /* Derivatives for TransferFcn: '<S1>/Transfer Fcn1' */
  _rtXdot->TransferFcn1_CSTATE = truesigma_P.TransferFcn1_A *
    truesigma_X.TransferFcn1_CSTATE;
  _rtXdot->TransferFcn1_CSTATE += truesigma_B.RateLimiter1;

  /* Derivatives for TransferFcn: '<S1>/Transfer Fcn2' */
  _rtXdot->TransferFcn2_CSTATE = truesigma_P.TransferFcn2_A *
    truesigma_X.TransferFcn2_CSTATE;
  _rtXdot->TransferFcn2_CSTATE += truesigma_B.RateLimiter2;

  /* Derivatives for EOM6DOFBodyEuler: '<Root>/Custom Variable Mass 6DOF (Euler Angles)' */
  memset(&_rtXdot->CustomVariableMass6DOFEulerAngl[0], 0, 12U * sizeof(real_T));
  tmp_3 = _mm_set_pd(truesigma_X.CustomVariableMass6DOFEulerAngl[6],
                     truesigma_X.CustomVariableMass6DOFEulerAngl[8]);
  tmp = _mm_sub_pd(_mm_div_pd(_mm_loadu_pd(&truesigma_B.forcesum[0]),
    _mm_set1_pd(truesigma_B.mass)), _mm_sub_pd(_mm_mul_pd(_mm_set_pd
    (truesigma_X.CustomVariableMass6DOFEulerAngl[0],
     truesigma_X.CustomVariableMass6DOFEulerAngl[2]), _mm_loadu_pd
    (&truesigma_X.CustomVariableMass6DOFEulerAngl[7])), _mm_mul_pd(_mm_loadu_pd(
    &truesigma_X.CustomVariableMass6DOFEulerAngl[1]), tmp_3)));
  _mm_storeu_pd(&_rtXdot->CustomVariableMass6DOFEulerAngl[0], tmp);
  _rtXdot->CustomVariableMass6DOFEulerAngl[2] = truesigma_B.forcesum[2] /
    truesigma_B.mass - (truesigma_X.CustomVariableMass6DOFEulerAngl[1] *
                        truesigma_X.CustomVariableMass6DOFEulerAngl[6] -
                        truesigma_X.CustomVariableMass6DOFEulerAngl[0] *
                        truesigma_X.CustomVariableMass6DOFEulerAngl[7]);
  pData_tmp_0 = cos(truesigma_X.CustomVariableMass6DOFEulerAngl[9]);
  pData_tmp = sin(truesigma_X.CustomVariableMass6DOFEulerAngl[9]);
  pData_tmp_2 = cos(truesigma_X.CustomVariableMass6DOFEulerAngl[10]);
  pData_tmp_1 = sin(truesigma_X.CustomVariableMass6DOFEulerAngl[10]);
  pData_idx_0 = 0.0 * pData_tmp_1 + pData_tmp_2;
  pData_idx_6_tmp = 0.0 * pData_tmp_2;
  pData_idx_6 = pData_idx_6_tmp - pData_tmp_1;
  pData_idx_1_tmp = pData_tmp_0 * 0.0;
  pData_idx_1_tmp_0 = pData_tmp * pData_tmp_1;
  pData_idx_1 = (pData_idx_6_tmp + pData_idx_1_tmp) + pData_idx_1_tmp_0;
  pData_idx_4 = pData_tmp * 0.0 + pData_tmp_0;
  pData_idx_7_tmp = 0.0 * -pData_tmp_1;
  pData_idx_7 = (pData_idx_7_tmp + pData_idx_1_tmp) + pData_tmp * pData_tmp_2;
  pData_idx_2_tmp = -pData_tmp * 0.0;
  pData_idx_2_tmp_0 = pData_tmp_1 * pData_tmp_0;
  pData_idx_6_tmp = (pData_idx_6_tmp + pData_idx_2_tmp) + pData_idx_2_tmp_0;
  pData_idx_1_tmp -= pData_tmp;
  pData_idx_7_tmp = (pData_idx_7_tmp + pData_idx_2_tmp) + pData_tmp_0 *
    pData_tmp_2;
  pData_tmp_1 = cos(truesigma_X.CustomVariableMass6DOFEulerAngl[11]);
  pData_idx_2_tmp = sin(truesigma_X.CustomVariableMass6DOFEulerAngl[11]);
  tmp = _mm_set1_pd(0.0);
  tmp_1 = _mm_set_pd(pData_idx_2_tmp, pData_tmp_1);
  tmp_2 = _mm_set_pd(pData_tmp_1, -pData_idx_2_tmp);
  _mm_storeu_pd(&_rtXdot->CustomVariableMass6DOFEulerAngl[3], _mm_add_pd
                (_mm_add_pd(_mm_mul_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
    (_mm_set1_pd(pData_idx_0), tmp_1), _mm_mul_pd(tmp, tmp_2)), _mm_mul_pd
    (_mm_set1_pd(pData_idx_6), tmp)), _mm_set1_pd
    (truesigma_X.CustomVariableMass6DOFEulerAngl[0])), _mm_mul_pd(_mm_add_pd
    (_mm_add_pd(_mm_mul_pd(tmp_1, _mm_set1_pd(pData_idx_1)), _mm_mul_pd
                (_mm_set_pd(pData_idx_4, -pData_idx_2_tmp), _mm_set_pd
                 (pData_tmp_1, pData_idx_4))), _mm_mul_pd(_mm_set1_pd
    (pData_idx_7), tmp)), _mm_set1_pd
    (truesigma_X.CustomVariableMass6DOFEulerAngl[1]))), _mm_mul_pd(_mm_add_pd
    (_mm_add_pd(_mm_mul_pd(tmp_1, _mm_set1_pd(pData_idx_6_tmp)), _mm_mul_pd
                (tmp_2, _mm_set1_pd(pData_idx_1_tmp))), _mm_mul_pd(_mm_set1_pd
    (pData_idx_7_tmp), tmp)), _mm_set1_pd
    (truesigma_X.CustomVariableMass6DOFEulerAngl[2]))));
  _rtXdot->CustomVariableMass6DOFEulerAngl[5] = (((pData_idx_1 * 0.0 +
    pData_idx_4 * 0.0) + pData_idx_7) *
    truesigma_X.CustomVariableMass6DOFEulerAngl[1] + (pData_idx_0 * 0.0 +
    pData_idx_6) * truesigma_X.CustomVariableMass6DOFEulerAngl[0]) +
    ((pData_idx_6_tmp * 0.0 + pData_idx_1_tmp * 0.0) + pData_idx_7_tmp) *
    truesigma_X.CustomVariableMass6DOFEulerAngl[2];
  tmp = _mm_set1_pd(truesigma_X.CustomVariableMass6DOFEulerAngl[6]);
  tmp_1 = _mm_set1_pd(truesigma_X.CustomVariableMass6DOFEulerAngl[7]);
  tmp_2 = _mm_set1_pd(truesigma_X.CustomVariableMass6DOFEulerAngl[8]);

  /* Derivatives for Constant: '<Root>/inertia' incorporates:
   *  EOM6DOFBodyEuler: '<Root>/Custom Variable Mass 6DOF (Euler Angles)'
   */
  _mm_storeu_pd(&tmp_0[0], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
    (&truesigma_P.inertia_Value[0]), tmp), _mm_mul_pd(_mm_loadu_pd
    (&truesigma_P.inertia_Value[3]), tmp_1)), _mm_mul_pd(_mm_loadu_pd
    (&truesigma_P.inertia_Value[6]), tmp_2)));

  /* Derivatives for EOM6DOFBodyEuler: '<Root>/Custom Variable Mass 6DOF (Euler Angles)' incorporates:
   *  Constant: '<Root>/Constant'
   *  Constant: '<Root>/inertia'
   * */
  pData_idx_0 = tmp_0[0];
  pData_idx_1 = tmp_0[1];
  pData_idx_6_tmp = (truesigma_P.inertia_Value[2] *
                     truesigma_X.CustomVariableMass6DOFEulerAngl[6] +
                     truesigma_P.inertia_Value[5] *
                     truesigma_X.CustomVariableMass6DOFEulerAngl[7]) +
    truesigma_P.inertia_Value[8] * truesigma_X.CustomVariableMass6DOFEulerAngl[8];
  pData_tmp_1 = truesigma_P.inertia_Value[4] * truesigma_P.inertia_Value[8] -
    truesigma_P.inertia_Value[5] * truesigma_P.inertia_Value[7];
  pData_idx_6 = truesigma_P.inertia_Value[1] * truesigma_P.inertia_Value[8] -
    truesigma_P.inertia_Value[2] * truesigma_P.inertia_Value[7];
  pData_idx_4 = truesigma_P.inertia_Value[1] * truesigma_P.inertia_Value[5] -
    truesigma_P.inertia_Value[2] * truesigma_P.inertia_Value[4];
  tmp_3 = _mm_sub_pd(_mm_sub_pd(_mm_loadu_pd(&truesigma_B.M_aero_body[0]),
    _mm_sub_pd(_mm_mul_pd(_mm_loadu_pd
    (&truesigma_X.CustomVariableMass6DOFEulerAngl[7]), _mm_set_pd(tmp_0[0],
    pData_idx_6_tmp)), _mm_mul_pd(tmp_3, _mm_set_pd(pData_idx_6_tmp, tmp_0[1])))),
                     _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
    (&truesigma_P.Constant_Value[0]), tmp), _mm_mul_pd(_mm_loadu_pd
    (&truesigma_P.Constant_Value[3]), tmp_1)), _mm_mul_pd(_mm_loadu_pd
    (&truesigma_P.Constant_Value[6]), tmp_2)));
  _mm_storeu_pd(&tmp_0[0], tmp_3);

  /* Derivatives for EOM6DOFBodyEuler: '<Root>/Custom Variable Mass 6DOF (Euler Angles)' incorporates:
   *  Constant: '<Root>/Constant'
   *  Constant: '<Root>/inertia'
   */
  pData_idx_6_tmp = (truesigma_B.M_aero_body[2] -
                     (truesigma_X.CustomVariableMass6DOFEulerAngl[6] *
                      pData_idx_1 - truesigma_X.CustomVariableMass6DOFEulerAngl
                      [7] * pData_idx_0)) - ((truesigma_P.Constant_Value[2] *
    truesigma_X.CustomVariableMass6DOFEulerAngl[6] + truesigma_P.Constant_Value
    [5] * truesigma_X.CustomVariableMass6DOFEulerAngl[7]) +
    truesigma_P.Constant_Value[8] * truesigma_X.CustomVariableMass6DOFEulerAngl
    [8]);
  pData_idx_0 = 1.0 / ((pData_tmp_1 * truesigma_P.inertia_Value[0] - pData_idx_6
                        * truesigma_P.inertia_Value[3]) + pData_idx_4 *
                       truesigma_P.inertia_Value[6]);
  _rtXdot->CustomVariableMass6DOFEulerAngl[6] = (-(truesigma_P.inertia_Value[3] *
    truesigma_P.inertia_Value[8] - truesigma_P.inertia_Value[5] *
    truesigma_P.inertia_Value[6]) * pData_idx_0 * tmp_0[1] + pData_tmp_1 *
    pData_idx_0 * tmp_0[0]) + (truesigma_P.inertia_Value[3] *
    truesigma_P.inertia_Value[7] - truesigma_P.inertia_Value[4] *
    truesigma_P.inertia_Value[6]) * pData_idx_0 * pData_idx_6_tmp;
  _rtXdot->CustomVariableMass6DOFEulerAngl[7] = ((truesigma_P.inertia_Value[0] *
    truesigma_P.inertia_Value[8] - truesigma_P.inertia_Value[2] *
    truesigma_P.inertia_Value[6]) * pData_idx_0 * tmp_0[1] + -pData_idx_6 *
    pData_idx_0 * tmp_0[0]) + -(truesigma_P.inertia_Value[0] *
    truesigma_P.inertia_Value[7] - truesigma_P.inertia_Value[1] *
    truesigma_P.inertia_Value[6]) * pData_idx_0 * pData_idx_6_tmp;
  _rtXdot->CustomVariableMass6DOFEulerAngl[8] = (-(truesigma_P.inertia_Value[0] *
    truesigma_P.inertia_Value[5] - truesigma_P.inertia_Value[2] *
    truesigma_P.inertia_Value[3]) * pData_idx_0 * tmp_0[1] + pData_idx_4 *
    pData_idx_0 * tmp_0[0]) + (truesigma_P.inertia_Value[0] *
    truesigma_P.inertia_Value[4] - truesigma_P.inertia_Value[1] *
    truesigma_P.inertia_Value[3]) * pData_idx_0 * pData_idx_6_tmp;
  _rtXdot->CustomVariableMass6DOFEulerAngl[9] = (pData_idx_1_tmp_0 / pData_tmp_2
    * truesigma_X.CustomVariableMass6DOFEulerAngl[7] +
    truesigma_X.CustomVariableMass6DOFEulerAngl[6]) + pData_idx_2_tmp_0 /
    pData_tmp_2 * truesigma_X.CustomVariableMass6DOFEulerAngl[8];
  pData_idx_0 = 0.0 * truesigma_X.CustomVariableMass6DOFEulerAngl[6];
  _rtXdot->CustomVariableMass6DOFEulerAngl[10] = (pData_tmp_0 *
    truesigma_X.CustomVariableMass6DOFEulerAngl[7] + pData_idx_0) + -pData_tmp *
    truesigma_X.CustomVariableMass6DOFEulerAngl[8];
  _rtXdot->CustomVariableMass6DOFEulerAngl[11] = (pData_tmp / pData_tmp_2 *
    truesigma_X.CustomVariableMass6DOFEulerAngl[7] + pData_idx_0) + pData_tmp_0 /
    pData_tmp_2 * truesigma_X.CustomVariableMass6DOFEulerAngl[8];
}

/* Model initialize function */
void truesigma_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)truesigma_M, 0,
                sizeof(RT_MODEL_truesigma_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&truesigma_M->solverInfo,
                          &truesigma_M->Timing.simTimeStep);
    rtsiSetTPtr(&truesigma_M->solverInfo, &rtmGetTPtr(truesigma_M));
    rtsiSetStepSizePtr(&truesigma_M->solverInfo, &truesigma_M->Timing.stepSize0);
    rtsiSetdXPtr(&truesigma_M->solverInfo, &truesigma_M->derivs);
    rtsiSetContStatesPtr(&truesigma_M->solverInfo, (real_T **)
                         &truesigma_M->contStates);
    rtsiSetNumContStatesPtr(&truesigma_M->solverInfo,
      &truesigma_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&truesigma_M->solverInfo,
      &truesigma_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&truesigma_M->solverInfo,
      &truesigma_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&truesigma_M->solverInfo,
      &truesigma_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&truesigma_M->solverInfo, (boolean_T**)
      &truesigma_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&truesigma_M->solverInfo, (&rtmGetErrorStatus
      (truesigma_M)));
    rtsiSetRTModelPtr(&truesigma_M->solverInfo, truesigma_M);
  }

  rtsiSetSimTimeStep(&truesigma_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&truesigma_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&truesigma_M->solverInfo, false);
  truesigma_M->intgData.y = truesigma_M->odeY;
  truesigma_M->intgData.f[0] = truesigma_M->odeF[0];
  truesigma_M->intgData.f[1] = truesigma_M->odeF[1];
  truesigma_M->intgData.f[2] = truesigma_M->odeF[2];
  truesigma_M->intgData.f[3] = truesigma_M->odeF[3];
  truesigma_M->contStates = ((X_truesigma_T *) &truesigma_X);
  truesigma_M->contStateDisabled = ((XDis_truesigma_T *) &truesigma_XDis);
  truesigma_M->Timing.tStart = (0.0);
  truesigma_M->periodicContStateIndices = ((int_T*) truesigma_PeriodicIndX);
  truesigma_M->periodicContStateRanges = ((real_T*) truesigma_PeriodicRngX);
  rtsiSetSolverData(&truesigma_M->solverInfo, (void *)&truesigma_M->intgData);
  rtsiSetSolverName(&truesigma_M->solverInfo,"ode4");
  rtmSetTPtr(truesigma_M, &truesigma_M->Timing.tArray[0]);
  rtmSetTFinal(truesigma_M, 15.0);
  truesigma_M->Timing.stepSize0 = 1.0;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    truesigma_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(truesigma_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(truesigma_M->rtwLogInfo, (NULL));
    rtliSetLogT(truesigma_M->rtwLogInfo, "tout");
    rtliSetLogX(truesigma_M->rtwLogInfo, "");
    rtliSetLogXFinal(truesigma_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(truesigma_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(truesigma_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(truesigma_M->rtwLogInfo, 0);
    rtliSetLogDecimation(truesigma_M->rtwLogInfo, 1);
    rtliSetLogY(truesigma_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(truesigma_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(truesigma_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &truesigma_B), 0,
                sizeof(B_truesigma_T));

  /* states (continuous) */
  {
    (void) memset((void *)&truesigma_X, 0,
                  sizeof(X_truesigma_T));
  }

  /* disabled states */
  {
    (void) memset((void *)&truesigma_XDis, 0,
                  sizeof(XDis_truesigma_T));
  }

  /* Periodic continuous states */
  {
    (void) memset((void*) truesigma_PeriodicIndX, 0,
                  3*sizeof(int_T));
    (void) memset((void*) truesigma_PeriodicRngX, 0,
                  6*sizeof(real_T));
  }

  /* states (dwork) */
  (void) memset((void *)&truesigma_DW, 0,
                sizeof(DW_truesigma_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(truesigma_M->rtwLogInfo, 0.0, rtmGetTFinal
    (truesigma_M), truesigma_M->Timing.stepSize0, (&rtmGetErrorStatus
    (truesigma_M)));

  /* InitializeConditions for TransferFcn: '<S1>/Transfer Fcn' */
  truesigma_X.TransferFcn_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S1>/Transfer Fcn1' */
  truesigma_X.TransferFcn1_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S1>/Transfer Fcn2' */
  truesigma_X.TransferFcn2_CSTATE = 0.0;

  /* InitializeConditions for EOM6DOFBodyEuler: '<Root>/Custom Variable Mass 6DOF (Euler Angles)' */
  truesigma_X.CustomVariableMass6DOFEulerAngl[0] =
    truesigma_P.CustomVariableMass6DOFEulerAn_o[0];
  truesigma_X.CustomVariableMass6DOFEulerAngl[3] =
    truesigma_P.CustomVariableMass6DOFEulerAngl[0];
  truesigma_X.CustomVariableMass6DOFEulerAngl[6] =
    truesigma_P.CustomVariableMass6DOFEulerAn_e[0];
  truesigma_X.CustomVariableMass6DOFEulerAngl[9] =
    truesigma_P.CustomVariableMass6DOFEulerAn_a[0];
  truesigma_X.CustomVariableMass6DOFEulerAngl[1] =
    truesigma_P.CustomVariableMass6DOFEulerAn_o[1];
  truesigma_X.CustomVariableMass6DOFEulerAngl[4] =
    truesigma_P.CustomVariableMass6DOFEulerAngl[1];
  truesigma_X.CustomVariableMass6DOFEulerAngl[7] =
    truesigma_P.CustomVariableMass6DOFEulerAn_e[1];
  truesigma_X.CustomVariableMass6DOFEulerAngl[10] =
    truesigma_P.CustomVariableMass6DOFEulerAn_a[1];
  truesigma_X.CustomVariableMass6DOFEulerAngl[2] =
    truesigma_P.CustomVariableMass6DOFEulerAn_o[2];
  truesigma_X.CustomVariableMass6DOFEulerAngl[5] =
    truesigma_P.CustomVariableMass6DOFEulerAngl[2];
  truesigma_X.CustomVariableMass6DOFEulerAngl[8] =
    truesigma_P.CustomVariableMass6DOFEulerAn_e[2];
  truesigma_X.CustomVariableMass6DOFEulerAngl[11] =
    truesigma_P.CustomVariableMass6DOFEulerAn_a[2];

  /* InitializeConditions for RateLimiter: '<S1>/Rate Limiter' */
  truesigma_DW.LastMajorTime = (rtInf);

  /* InitializeConditions for RateLimiter: '<S1>/Rate Limiter1' */
  truesigma_DW.LastMajorTime_h = (rtInf);

  /* InitializeConditions for RateLimiter: '<S1>/Rate Limiter2' */
  truesigma_DW.LastMajorTime_m = (rtInf);

  /* InitializeConditions for root-level periodic continuous states */
  {
    int_T rootPeriodicContStateIndices[3] = { 0, 0, 0 };

    real_T rootPeriodicContStateRanges[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    (void) memcpy((void*)truesigma_PeriodicIndX, rootPeriodicContStateIndices,
                  3*sizeof(int_T));
    (void) memcpy((void*)truesigma_PeriodicRngX, rootPeriodicContStateRanges,
                  6*sizeof(real_T));
  }
}

/* Model terminate function */
void truesigma_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * truesigma_data.c
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

/* Block parameters (default storage) */
P_truesigma_T truesigma_P = {
  /* Computed Parameter: TransferFcn_A
   * Referenced by: '<S1>/Transfer Fcn'
   */
  -33.333333333333336,

  /* Computed Parameter: TransferFcn_C
   * Referenced by: '<S1>/Transfer Fcn'
   */
  33.333333333333336,

  /* Expression: 0.26
   * Referenced by: '<S1>/Saturation'
   */
  0.26,

  /* Expression: -0.26
   * Referenced by: '<S1>/Saturation'
   */
  -0.26,

  /* Computed Parameter: TransferFcn1_A
   * Referenced by: '<S1>/Transfer Fcn1'
   */
  -33.333333333333336,

  /* Computed Parameter: TransferFcn1_C
   * Referenced by: '<S1>/Transfer Fcn1'
   */
  33.333333333333336,

  /* Expression: 0.26
   * Referenced by: '<S1>/Saturation1'
   */
  0.26,

  /* Expression: -0.26
   * Referenced by: '<S1>/Saturation1'
   */
  -0.26,

  /* Computed Parameter: TransferFcn2_A
   * Referenced by: '<S1>/Transfer Fcn2'
   */
  -33.333333333333336,

  /* Computed Parameter: TransferFcn2_C
   * Referenced by: '<S1>/Transfer Fcn2'
   */
  33.333333333333336,

  /* Expression: 0.26
   * Referenced by: '<S1>/Saturation2'
   */
  0.26,

  /* Expression: -0.26
   * Referenced by: '<S1>/Saturation2'
   */
  -0.26,

  /* Expression: [0 0 0]
   * Referenced by: '<Root>/Custom Variable Mass 6DOF (Euler Angles)'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [0 0 0]
   * Referenced by: '<Root>/Custom Variable Mass 6DOF (Euler Angles)'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [0 pi/2 0]
   * Referenced by: '<Root>/Custom Variable Mass 6DOF (Euler Angles)'
   */
  { 0.0, 1.5707963267948966, 0.0 },

  /* Expression: [0 0 0]
   * Referenced by: '<Root>/Custom Variable Mass 6DOF (Euler Angles)'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [150; 0; -300]
   * Referenced by: '<Root>/waypoint'
   */
  { 150.0, 0.0, -300.0 },

  /* Expression: zeros(3)
   * Referenced by: '<Root>/Constant'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: 2
   * Referenced by: '<Root>/Step1'
   */
  2.0,

  /* Expression: 0
   * Referenced by: '<Root>/Step1'
   */
  0.0,

  /* Expression: -37.5
   * Referenced by: '<Root>/Step1'
   */
  -37.5,

  /* Expression: 10
   * Referenced by: '<S1>/Rate Limiter'
   */
  10.0,

  /* Expression: -10
   * Referenced by: '<S1>/Rate Limiter'
   */
  -10.0,

  /* Expression: 10
   * Referenced by: '<S1>/Rate Limiter1'
   */
  10.0,

  /* Expression: -10
   * Referenced by: '<S1>/Rate Limiter1'
   */
  -10.0,

  /* Expression: 10
   * Referenced by: '<S1>/Rate Limiter2'
   */
  10.0,

  /* Expression: -10
   * Referenced by: '<S1>/Rate Limiter2'
   */
  -10.0,

  /* Expression: 0
   * Referenced by: '<Root>/Thrust'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Thrust'
   */
  0.0,

  /* Expression: 37.5
   * Referenced by: '<Root>/Thrust'
   */
  37.5,

  /* Expression: diag([0.0002, 0.01, 0.0146])
   * Referenced by: '<Root>/inertia'
   */
  { 0.0002, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0146 }
};

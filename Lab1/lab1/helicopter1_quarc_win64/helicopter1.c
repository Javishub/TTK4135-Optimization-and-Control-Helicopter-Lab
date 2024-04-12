/*
 * helicopter1.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "helicopter1".
 *
 * Model version              : 11.4
 * Simulink Coder version : 9.4 (R2020b) 29-Jul-2020
 * C source code generated on : Tue Jan 30 15:15:18 2024
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "helicopter1.h"
#include "helicopter1_private.h"

/* Block signals (default storage) */
B_helicopter1_T helicopter1_B;

/* Continuous states */
X_helicopter1_T helicopter1_X;

/* Block states (default storage) */
DW_helicopter1_T helicopter1_DW;

/* Real-time model */
static RT_MODEL_helicopter1_T helicopter1_M_;
RT_MODEL_helicopter1_T *const helicopter1_M = &helicopter1_M_;

/*
 * This function updates continuous states using the ODE1 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE1_IntgData *id = (ODE1_IntgData *)rtsiGetSolverData(si);
  real_T *f0 = id->f[0];
  int_T i;
  int_T nXc = 4;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  helicopter1_derivatives();
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model output function */
void helicopter1_output(void)
{
  /* local block i/o variables */
  real_T rtb_HILReadEncoderTimebase_o1;
  real_T rtb_HILReadEncoderTimebase_o2;
  real_T rtb_HILReadEncoderTimebase_o3;
  real_T rtb_Clock;
  real_T rtb_Frontgain;
  real_T rtb_Sum;
  int8_T rtAction;
  if (rtmIsMajorTimeStep(helicopter1_M)) {
    /* set solver stop time */
    if (!(helicopter1_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&helicopter1_M->solverInfo,
                            ((helicopter1_M->Timing.clockTickH0 + 1) *
        helicopter1_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&helicopter1_M->solverInfo,
                            ((helicopter1_M->Timing.clockTick0 + 1) *
        helicopter1_M->Timing.stepSize0 + helicopter1_M->Timing.clockTickH0 *
        helicopter1_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(helicopter1_M)) {
    helicopter1_M->Timing.t[0] = rtsiGetT(&helicopter1_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(helicopter1_M)) {
    /* S-Function (hil_read_encoder_timebase_block): '<S4>/HIL Read Encoder Timebase' */

    /* S-Function Block: helicopter1/Helicopter_interface/HIL Read Encoder Timebase (hil_read_encoder_timebase_block) */
    {
      t_error result;
      result = hil_task_read_encoder(helicopter1_DW.HILReadEncoderTimebase_Task,
        1, &helicopter1_DW.HILReadEncoderTimebase_Buffer[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(helicopter1_M, _rt_error_message);
      } else {
        rtb_HILReadEncoderTimebase_o1 =
          helicopter1_DW.HILReadEncoderTimebase_Buffer[0];
        rtb_HILReadEncoderTimebase_o2 =
          helicopter1_DW.HILReadEncoderTimebase_Buffer[1];
        rtb_HILReadEncoderTimebase_o3 =
          helicopter1_DW.HILReadEncoderTimebase_Buffer[2];
      }
    }

    /* Gain: '<S4>/Travel: Count to rad' incorporates:
     *  Gain: '<S4>/Travel_gain'
     */
    helicopter1_B.TravelCounttorad = helicopter1_P.travel_gain *
      rtb_HILReadEncoderTimebase_o1 * helicopter1_P.TravelCounttorad_Gain;

    /* Gain: '<S12>/Gain' */
    helicopter1_B.Gain = helicopter1_P.Gain_Gain *
      helicopter1_B.TravelCounttorad;

    /* Gain: '<S4>/Pitch: Count to rad' */
    helicopter1_B.PitchCounttorad = helicopter1_P.PitchCounttorad_Gain *
      rtb_HILReadEncoderTimebase_o2;

    /* Gain: '<S9>/Gain' */
    helicopter1_B.Gain_i = helicopter1_P.Gain_Gain_a *
      helicopter1_B.PitchCounttorad;

    /* Gain: '<S4>/Elevation: Count to rad' incorporates:
     *  Gain: '<S4>/Elevation_gain'
     */
    helicopter1_B.ElevationCounttorad = helicopter1_P.elevation_gain *
      rtb_HILReadEncoderTimebase_o3 * helicopter1_P.ElevationCounttorad_Gain;

    /* Sum: '<Root>/Sum' incorporates:
     *  Constant: '<Root>/elavation_offset [deg]'
     *  Gain: '<S7>/Gain'
     */
    helicopter1_B.Sum = helicopter1_P.Gain_Gain_lv *
      helicopter1_B.ElevationCounttorad +
      helicopter1_P.elavation_offsetdeg_Value;
  }

  /* Sum: '<Root>/Sum1' incorporates:
   *  Constant: '<Root>/Vd_bias'
   *  Constant: '<Root>/pitch_ref'
   *  Gain: '<S10>/Gain'
   *  Gain: '<S2>/Gain1'
   *  Gain: '<S5>/K_pd'
   *  Gain: '<S5>/K_pp'
   *  Sum: '<S5>/Sum2'
   *  Sum: '<S5>/Sum3'
   *  TransferFcn: '<S4>/Pitch: Transfer Fcn'
   */
  rtb_Frontgain = ((helicopter1_P.pitch_ref_Value - helicopter1_P.Gain1_Gain *
                    helicopter1_B.Gain_i) * helicopter1_P.K_pp -
                   (helicopter1_P.PitchTransferFcn_C *
                    helicopter1_X.PitchTransferFcn_CSTATE +
                    helicopter1_P.PitchTransferFcn_D *
                    helicopter1_B.PitchCounttorad) * helicopter1_P.Gain_Gain_ae *
                   helicopter1_P.Gain1_Gain * helicopter1_P.K_pd) +
    helicopter1_P.Vd_ff;

  /* Integrator: '<S3>/Integrator' */
  /* Limited  Integrator  */
  if (helicopter1_X.Integrator_CSTATE >= helicopter1_P.Integrator_UpperSat) {
    helicopter1_X.Integrator_CSTATE = helicopter1_P.Integrator_UpperSat;
  } else {
    if (helicopter1_X.Integrator_CSTATE <= helicopter1_P.Integrator_LowerSat) {
      helicopter1_X.Integrator_CSTATE = helicopter1_P.Integrator_LowerSat;
    }
  }

  /* Sum: '<S3>/Sum' incorporates:
   *  Constant: '<Root>/elevation_ref'
   *  Gain: '<S2>/Gain1'
   */
  rtb_Sum = helicopter1_P.elevation_ref_Value - helicopter1_P.Gain1_Gain *
    helicopter1_B.Sum;

  /* Sum: '<Root>/Sum2' incorporates:
   *  Constant: '<Root>/Vs_bias'
   *  Gain: '<S2>/Gain1'
   *  Gain: '<S3>/K_ed'
   *  Gain: '<S3>/K_ep'
   *  Gain: '<S8>/Gain'
   *  Integrator: '<S3>/Integrator'
   *  Sum: '<S3>/Sum1'
   *  TransferFcn: '<S4>/Elevation: Transfer Fcn'
   */
  rtb_Clock = ((helicopter1_P.K_ep * rtb_Sum + helicopter1_X.Integrator_CSTATE)
               - (helicopter1_P.ElevationTransferFcn_C *
                  helicopter1_X.ElevationTransferFcn_CSTATE +
                  helicopter1_P.ElevationTransferFcn_D *
                  helicopter1_B.ElevationCounttorad) * helicopter1_P.Gain_Gain_n
               * helicopter1_P.Gain1_Gain * helicopter1_P.K_ed) +
    helicopter1_P.Vs_ff;

  /* If: '<S3>/If' incorporates:
   *  Clock: '<S3>/Clock'
   *  Gain: '<S3>/K_ei'
   *  Inport: '<S6>/In1'
   */
  if (rtmIsMajorTimeStep(helicopter1_M)) {
    rtAction = (int8_T)!(helicopter1_M->Timing.t[0] >= 2.0);
    helicopter1_DW.If_ActiveSubsystem = rtAction;
  } else {
    rtAction = helicopter1_DW.If_ActiveSubsystem;
  }

  if (rtAction == 0) {
    /* Outputs for IfAction SubSystem: '<S3>/If Action Subsystem' incorporates:
     *  ActionPort: '<S6>/Action Port'
     */
    helicopter1_B.In1 = helicopter1_P.K_ei * rtb_Sum;

    /* End of Outputs for SubSystem: '<S3>/If Action Subsystem' */
  }

  /* End of If: '<S3>/If' */

  /* Gain: '<S1>/Front gain' incorporates:
   *  Sum: '<S1>/Add'
   */
  rtb_Sum = (rtb_Frontgain + rtb_Clock) * helicopter1_P.Frontgain_Gain;

  /* Saturate: '<S4>/Front motor: Saturation' */
  if (rtb_Sum > helicopter1_P.FrontmotorSaturation_UpperSat) {
    /* Saturate: '<S4>/Front motor: Saturation' */
    helicopter1_B.FrontmotorSaturation =
      helicopter1_P.FrontmotorSaturation_UpperSat;
  } else if (rtb_Sum < helicopter1_P.FrontmotorSaturation_LowerSat) {
    /* Saturate: '<S4>/Front motor: Saturation' */
    helicopter1_B.FrontmotorSaturation =
      helicopter1_P.FrontmotorSaturation_LowerSat;
  } else {
    /* Saturate: '<S4>/Front motor: Saturation' */
    helicopter1_B.FrontmotorSaturation = rtb_Sum;
  }

  /* End of Saturate: '<S4>/Front motor: Saturation' */

  /* Gain: '<S1>/Back gain' incorporates:
   *  Sum: '<S1>/Subtract'
   */
  rtb_Sum = (rtb_Clock - rtb_Frontgain) * helicopter1_P.Backgain_Gain;

  /* Saturate: '<S4>/Back motor: Saturation' */
  if (rtb_Sum > helicopter1_P.BackmotorSaturation_UpperSat) {
    /* Saturate: '<S4>/Back motor: Saturation' */
    helicopter1_B.BackmotorSaturation =
      helicopter1_P.BackmotorSaturation_UpperSat;
  } else if (rtb_Sum < helicopter1_P.BackmotorSaturation_LowerSat) {
    /* Saturate: '<S4>/Back motor: Saturation' */
    helicopter1_B.BackmotorSaturation =
      helicopter1_P.BackmotorSaturation_LowerSat;
  } else {
    /* Saturate: '<S4>/Back motor: Saturation' */
    helicopter1_B.BackmotorSaturation = rtb_Sum;
  }

  /* End of Saturate: '<S4>/Back motor: Saturation' */
  if (rtmIsMajorTimeStep(helicopter1_M)) {
    /* S-Function (hil_write_analog_block): '<S4>/HIL Write Analog' */

    /* S-Function Block: helicopter1/Helicopter_interface/HIL Write Analog (hil_write_analog_block) */
    {
      t_error result;
      helicopter1_DW.HILWriteAnalog_Buffer[0] =
        helicopter1_B.FrontmotorSaturation;
      helicopter1_DW.HILWriteAnalog_Buffer[1] =
        helicopter1_B.BackmotorSaturation;
      result = hil_write_analog(helicopter1_DW.HILInitialize_Card,
        helicopter1_P.HILWriteAnalog_channels, 2,
        &helicopter1_DW.HILWriteAnalog_Buffer[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(helicopter1_M, _rt_error_message);
      }
    }
  }
}

/* Model update function */
void helicopter1_update(void)
{
  if (rtmIsMajorTimeStep(helicopter1_M)) {
    rt_ertODEUpdateContinuousStates(&helicopter1_M->solverInfo);
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++helicopter1_M->Timing.clockTick0)) {
    ++helicopter1_M->Timing.clockTickH0;
  }

  helicopter1_M->Timing.t[0] = rtsiGetSolverStopTime(&helicopter1_M->solverInfo);

  {
    /* Update absolute timer for sample time: [0.002s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++helicopter1_M->Timing.clockTick1)) {
      ++helicopter1_M->Timing.clockTickH1;
    }

    helicopter1_M->Timing.t[1] = helicopter1_M->Timing.clockTick1 *
      helicopter1_M->Timing.stepSize1 + helicopter1_M->Timing.clockTickH1 *
      helicopter1_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Derivatives for root system: '<Root>' */
void helicopter1_derivatives(void)
{
  XDot_helicopter1_T *_rtXdot;
  boolean_T lsat;
  boolean_T usat;
  _rtXdot = ((XDot_helicopter1_T *) helicopter1_M->derivs);

  /* Derivatives for TransferFcn: '<S4>/Travel: Transfer Fcn' */
  _rtXdot->TravelTransferFcn_CSTATE = 0.0;
  _rtXdot->TravelTransferFcn_CSTATE += helicopter1_P.TravelTransferFcn_A *
    helicopter1_X.TravelTransferFcn_CSTATE;
  _rtXdot->TravelTransferFcn_CSTATE += helicopter1_B.TravelCounttorad;

  /* Derivatives for TransferFcn: '<S4>/Pitch: Transfer Fcn' */
  _rtXdot->PitchTransferFcn_CSTATE = 0.0;
  _rtXdot->PitchTransferFcn_CSTATE += helicopter1_P.PitchTransferFcn_A *
    helicopter1_X.PitchTransferFcn_CSTATE;
  _rtXdot->PitchTransferFcn_CSTATE += helicopter1_B.PitchCounttorad;

  /* Derivatives for TransferFcn: '<S4>/Elevation: Transfer Fcn' */
  _rtXdot->ElevationTransferFcn_CSTATE = 0.0;
  _rtXdot->ElevationTransferFcn_CSTATE += helicopter1_P.ElevationTransferFcn_A *
    helicopter1_X.ElevationTransferFcn_CSTATE;
  _rtXdot->ElevationTransferFcn_CSTATE += helicopter1_B.ElevationCounttorad;

  /* Derivatives for Integrator: '<S3>/Integrator' */
  lsat = (helicopter1_X.Integrator_CSTATE <= helicopter1_P.Integrator_LowerSat);
  usat = (helicopter1_X.Integrator_CSTATE >= helicopter1_P.Integrator_UpperSat);
  if (((!lsat) && (!usat)) || (lsat && (helicopter1_B.In1 > 0.0)) || (usat &&
       (helicopter1_B.In1 < 0.0))) {
    _rtXdot->Integrator_CSTATE = helicopter1_B.In1;
  } else {
    /* in saturation */
    _rtXdot->Integrator_CSTATE = 0.0;
  }

  /* End of Derivatives for Integrator: '<S3>/Integrator' */
}

/* Model initialize function */
void helicopter1_initialize(void)
{
  /* Start for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: helicopter1/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("q8_usb", "0", &helicopter1_DW.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(helicopter1_M, _rt_error_message);
      return;
    }

    is_switching = false;
    result = hil_set_card_specific_options(helicopter1_DW.HILInitialize_Card,
      "update_rate=normal;decimation=1", 32);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(helicopter1_M, _rt_error_message);
      return;
    }

    result = hil_watchdog_clear(helicopter1_DW.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(helicopter1_M, _rt_error_message);
      return;
    }

    if ((helicopter1_P.HILInitialize_AIPStart && !is_switching) ||
        (helicopter1_P.HILInitialize_AIPEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_AIMinimums = &helicopter1_DW.HILInitialize_AIMinimums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AIMinimums[i1] = (helicopter1_P.HILInitialize_AILow);
        }
      }

      {
        int_T i1;
        real_T *dw_AIMaximums = &helicopter1_DW.HILInitialize_AIMaximums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AIMaximums[i1] = helicopter1_P.HILInitialize_AIHigh;
        }
      }

      result = hil_set_analog_input_ranges(helicopter1_DW.HILInitialize_Card,
        helicopter1_P.HILInitialize_AIChannels, 8U,
        &helicopter1_DW.HILInitialize_AIMinimums[0],
        &helicopter1_DW.HILInitialize_AIMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(helicopter1_M, _rt_error_message);
        return;
      }
    }

    if ((helicopter1_P.HILInitialize_AOPStart && !is_switching) ||
        (helicopter1_P.HILInitialize_AOPEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOMinimums = &helicopter1_DW.HILInitialize_AOMinimums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOMinimums[i1] = (helicopter1_P.HILInitialize_AOLow);
        }
      }

      {
        int_T i1;
        real_T *dw_AOMaximums = &helicopter1_DW.HILInitialize_AOMaximums[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOMaximums[i1] = helicopter1_P.HILInitialize_AOHigh;
        }
      }

      result = hil_set_analog_output_ranges(helicopter1_DW.HILInitialize_Card,
        helicopter1_P.HILInitialize_AOChannels, 8U,
        &helicopter1_DW.HILInitialize_AOMinimums[0],
        &helicopter1_DW.HILInitialize_AOMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(helicopter1_M, _rt_error_message);
        return;
      }
    }

    if ((helicopter1_P.HILInitialize_AOStart && !is_switching) ||
        (helicopter1_P.HILInitialize_AOEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOVoltages = &helicopter1_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = helicopter1_P.HILInitialize_AOInitial;
        }
      }

      result = hil_write_analog(helicopter1_DW.HILInitialize_Card,
        helicopter1_P.HILInitialize_AOChannels, 8U,
        &helicopter1_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(helicopter1_M, _rt_error_message);
        return;
      }
    }

    if (helicopter1_P.HILInitialize_AOReset) {
      {
        int_T i1;
        real_T *dw_AOVoltages = &helicopter1_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = helicopter1_P.HILInitialize_AOWatchdog;
        }
      }

      result = hil_watchdog_set_analog_expiration_state
        (helicopter1_DW.HILInitialize_Card,
         helicopter1_P.HILInitialize_AOChannels, 8U,
         &helicopter1_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(helicopter1_M, _rt_error_message);
        return;
      }
    }

    if ((helicopter1_P.HILInitialize_EIPStart && !is_switching) ||
        (helicopter1_P.HILInitialize_EIPEnter && is_switching)) {
      {
        int_T i1;
        int32_T *dw_QuadratureModes =
          &helicopter1_DW.HILInitialize_QuadratureModes[0];
        for (i1=0; i1 < 8; i1++) {
          dw_QuadratureModes[i1] = helicopter1_P.HILInitialize_EIQuadrature;
        }
      }

      result = hil_set_encoder_quadrature_mode(helicopter1_DW.HILInitialize_Card,
        helicopter1_P.HILInitialize_EIChannels, 8U, (t_encoder_quadrature_mode *)
        &helicopter1_DW.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(helicopter1_M, _rt_error_message);
        return;
      }
    }

    if ((helicopter1_P.HILInitialize_EIStart && !is_switching) ||
        (helicopter1_P.HILInitialize_EIEnter && is_switching)) {
      {
        int_T i1;
        int32_T *dw_InitialEICounts =
          &helicopter1_DW.HILInitialize_InitialEICounts[0];
        for (i1=0; i1 < 8; i1++) {
          dw_InitialEICounts[i1] = helicopter1_P.HILInitialize_EIInitial;
        }
      }

      result = hil_set_encoder_counts(helicopter1_DW.HILInitialize_Card,
        helicopter1_P.HILInitialize_EIChannels, 8U,
        &helicopter1_DW.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(helicopter1_M, _rt_error_message);
        return;
      }
    }

    if ((helicopter1_P.HILInitialize_POPStart && !is_switching) ||
        (helicopter1_P.HILInitialize_POPEnter && is_switching)) {
      uint32_T num_duty_cycle_modes = 0;
      uint32_T num_frequency_modes = 0;

      {
        int_T i1;
        int32_T *dw_POModeValues = &helicopter1_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POModeValues[i1] = helicopter1_P.HILInitialize_POModes;
        }
      }

      result = hil_set_pwm_mode(helicopter1_DW.HILInitialize_Card,
        helicopter1_P.HILInitialize_POChannels, 8U, (t_pwm_mode *)
        &helicopter1_DW.HILInitialize_POModeValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(helicopter1_M, _rt_error_message);
        return;
      }

      {
        int_T i1;
        const uint32_T *p_HILInitialize_POChannels =
          helicopter1_P.HILInitialize_POChannels;
        int32_T *dw_POModeValues = &helicopter1_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          if (dw_POModeValues[i1] == PWM_DUTY_CYCLE_MODE || dw_POModeValues[i1] ==
              PWM_ONE_SHOT_MODE || dw_POModeValues[i1] == PWM_TIME_MODE ||
              dw_POModeValues[i1] == PWM_RAW_MODE) {
            helicopter1_DW.HILInitialize_POSortedChans[num_duty_cycle_modes] =
              (p_HILInitialize_POChannels[i1]);
            helicopter1_DW.HILInitialize_POSortedFreqs[num_duty_cycle_modes] =
              helicopter1_P.HILInitialize_POFrequency;
            num_duty_cycle_modes++;
          } else {
            helicopter1_DW.HILInitialize_POSortedChans[7U - num_frequency_modes]
              = (p_HILInitialize_POChannels[i1]);
            helicopter1_DW.HILInitialize_POSortedFreqs[7U - num_frequency_modes]
              = helicopter1_P.HILInitialize_POFrequency;
            num_frequency_modes++;
          }
        }
      }

      if (num_duty_cycle_modes > 0) {
        result = hil_set_pwm_frequency(helicopter1_DW.HILInitialize_Card,
          &helicopter1_DW.HILInitialize_POSortedChans[0], num_duty_cycle_modes,
          &helicopter1_DW.HILInitialize_POSortedFreqs[0]);
        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(helicopter1_M, _rt_error_message);
          return;
        }
      }

      if (num_frequency_modes > 0) {
        result = hil_set_pwm_duty_cycle(helicopter1_DW.HILInitialize_Card,
          &helicopter1_DW.HILInitialize_POSortedChans[num_duty_cycle_modes],
          num_frequency_modes,
          &helicopter1_DW.HILInitialize_POSortedFreqs[num_duty_cycle_modes]);
        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(helicopter1_M, _rt_error_message);
          return;
        }
      }

      {
        int_T i1;
        int32_T *dw_POModeValues = &helicopter1_DW.HILInitialize_POModeValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POModeValues[i1] = helicopter1_P.HILInitialize_POConfiguration;
        }
      }

      {
        int_T i1;
        int32_T *dw_POAlignValues = &helicopter1_DW.HILInitialize_POAlignValues
          [0];
        for (i1=0; i1 < 8; i1++) {
          dw_POAlignValues[i1] = helicopter1_P.HILInitialize_POAlignment;
        }
      }

      {
        int_T i1;
        int32_T *dw_POPolarityVals =
          &helicopter1_DW.HILInitialize_POPolarityVals[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POPolarityVals[i1] = helicopter1_P.HILInitialize_POPolarity;
        }
      }

      result = hil_set_pwm_configuration(helicopter1_DW.HILInitialize_Card,
        helicopter1_P.HILInitialize_POChannels, 8U,
        (t_pwm_configuration *) &helicopter1_DW.HILInitialize_POModeValues[0],
        (t_pwm_alignment *) &helicopter1_DW.HILInitialize_POAlignValues[0],
        (t_pwm_polarity *) &helicopter1_DW.HILInitialize_POPolarityVals[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(helicopter1_M, _rt_error_message);
        return;
      }

      {
        int_T i1;
        real_T *dw_POSortedFreqs = &helicopter1_DW.HILInitialize_POSortedFreqs[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POSortedFreqs[i1] = helicopter1_P.HILInitialize_POLeading;
        }
      }

      {
        int_T i1;
        real_T *dw_POValues = &helicopter1_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = helicopter1_P.HILInitialize_POTrailing;
        }
      }

      result = hil_set_pwm_deadband(helicopter1_DW.HILInitialize_Card,
        helicopter1_P.HILInitialize_POChannels, 8U,
        &helicopter1_DW.HILInitialize_POSortedFreqs[0],
        &helicopter1_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(helicopter1_M, _rt_error_message);
        return;
      }
    }

    if ((helicopter1_P.HILInitialize_POStart && !is_switching) ||
        (helicopter1_P.HILInitialize_POEnter && is_switching)) {
      {
        int_T i1;
        real_T *dw_POValues = &helicopter1_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = helicopter1_P.HILInitialize_POInitial;
        }
      }

      result = hil_write_pwm(helicopter1_DW.HILInitialize_Card,
        helicopter1_P.HILInitialize_POChannels, 8U,
        &helicopter1_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(helicopter1_M, _rt_error_message);
        return;
      }
    }

    if (helicopter1_P.HILInitialize_POReset) {
      {
        int_T i1;
        real_T *dw_POValues = &helicopter1_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = helicopter1_P.HILInitialize_POWatchdog;
        }
      }

      result = hil_watchdog_set_pwm_expiration_state
        (helicopter1_DW.HILInitialize_Card,
         helicopter1_P.HILInitialize_POChannels, 8U,
         &helicopter1_DW.HILInitialize_POValues[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(helicopter1_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for S-Function (hil_read_encoder_timebase_block): '<S4>/HIL Read Encoder Timebase' */

  /* S-Function Block: helicopter1/Helicopter_interface/HIL Read Encoder Timebase (hil_read_encoder_timebase_block) */
  {
    t_error result;
    result = hil_task_create_encoder_reader(helicopter1_DW.HILInitialize_Card,
      helicopter1_P.HILReadEncoderTimebase_SamplesI,
      helicopter1_P.HILReadEncoderTimebase_Channels, 3,
      &helicopter1_DW.HILReadEncoderTimebase_Task);
    if (result >= 0) {
      result = hil_task_set_buffer_overflow_mode
        (helicopter1_DW.HILReadEncoderTimebase_Task, (t_buffer_overflow_mode)
         (helicopter1_P.HILReadEncoderTimebase_Overflow - 1));
    }

    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(helicopter1_M, _rt_error_message);
    }
  }

  /* Start for If: '<S3>/If' */
  helicopter1_DW.If_ActiveSubsystem = -1;

  /* InitializeConditions for TransferFcn: '<S4>/Travel: Transfer Fcn' */
  helicopter1_X.TravelTransferFcn_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S4>/Pitch: Transfer Fcn' */
  helicopter1_X.PitchTransferFcn_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S4>/Elevation: Transfer Fcn' */
  helicopter1_X.ElevationTransferFcn_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S3>/Integrator' */
  helicopter1_X.Integrator_CSTATE = helicopter1_P.Integrator_IC;
}

/* Model terminate function */
void helicopter1_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: helicopter1/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    t_uint32 num_final_analog_outputs = 0;
    t_uint32 num_final_pwm_outputs = 0;
    hil_task_stop_all(helicopter1_DW.HILInitialize_Card);
    hil_monitor_stop_all(helicopter1_DW.HILInitialize_Card);
    is_switching = false;
    if ((helicopter1_P.HILInitialize_AOTerminate && !is_switching) ||
        (helicopter1_P.HILInitialize_AOExit && is_switching)) {
      {
        int_T i1;
        real_T *dw_AOVoltages = &helicopter1_DW.HILInitialize_AOVoltages[0];
        for (i1=0; i1 < 8; i1++) {
          dw_AOVoltages[i1] = helicopter1_P.HILInitialize_AOFinal;
        }
      }

      num_final_analog_outputs = 8U;
    } else {
      num_final_analog_outputs = 0;
    }

    if ((helicopter1_P.HILInitialize_POTerminate && !is_switching) ||
        (helicopter1_P.HILInitialize_POExit && is_switching)) {
      {
        int_T i1;
        real_T *dw_POValues = &helicopter1_DW.HILInitialize_POValues[0];
        for (i1=0; i1 < 8; i1++) {
          dw_POValues[i1] = helicopter1_P.HILInitialize_POFinal;
        }
      }

      num_final_pwm_outputs = 8U;
    } else {
      num_final_pwm_outputs = 0;
    }

    if (0
        || num_final_analog_outputs > 0
        || num_final_pwm_outputs > 0
        ) {
      /* Attempt to write the final outputs atomically (due to firmware issue in old Q2-USB). Otherwise write channels individually */
      result = hil_write(helicopter1_DW.HILInitialize_Card
                         , helicopter1_P.HILInitialize_AOChannels,
                         num_final_analog_outputs
                         , helicopter1_P.HILInitialize_POChannels,
                         num_final_pwm_outputs
                         , NULL, 0
                         , NULL, 0
                         , &helicopter1_DW.HILInitialize_AOVoltages[0]
                         , &helicopter1_DW.HILInitialize_POValues[0]
                         , (t_boolean *) NULL
                         , NULL
                         );
      if (result == -QERR_HIL_WRITE_NOT_SUPPORTED) {
        t_error local_result;
        result = 0;

        /* The hil_write operation is not supported by this card. Write final outputs for each channel type */
        if (num_final_analog_outputs > 0) {
          local_result = hil_write_analog(helicopter1_DW.HILInitialize_Card,
            helicopter1_P.HILInitialize_AOChannels, num_final_analog_outputs,
            &helicopter1_DW.HILInitialize_AOVoltages[0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_pwm_outputs > 0) {
          local_result = hil_write_pwm(helicopter1_DW.HILInitialize_Card,
            helicopter1_P.HILInitialize_POChannels, num_final_pwm_outputs,
            &helicopter1_DW.HILInitialize_POValues[0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(helicopter1_M, _rt_error_message);
        }
      }
    }

    hil_task_delete_all(helicopter1_DW.HILInitialize_Card);
    hil_monitor_delete_all(helicopter1_DW.HILInitialize_Card);
    hil_close(helicopter1_DW.HILInitialize_Card);
    helicopter1_DW.HILInitialize_Card = NULL;
  }
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/

/* Solver interface called by GRT_Main */
#ifndef USE_GENERATED_SOLVER

void rt_ODECreateIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEDestroyIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEUpdateContinuousStates(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

#endif

void MdlOutputs(int_T tid)
{
  helicopter1_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  helicopter1_update();
  UNUSED_PARAMETER(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  helicopter1_initialize();
}

void MdlTerminate(void)
{
  helicopter1_terminate();
}

/* Registration function */
RT_MODEL_helicopter1_T *helicopter1(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  helicopter1_P.Integrator_UpperSat = rtInf;
  helicopter1_P.Integrator_LowerSat = rtMinusInf;

  /* initialize real-time model */
  (void) memset((void *)helicopter1_M, 0,
                sizeof(RT_MODEL_helicopter1_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&helicopter1_M->solverInfo,
                          &helicopter1_M->Timing.simTimeStep);
    rtsiSetTPtr(&helicopter1_M->solverInfo, &rtmGetTPtr(helicopter1_M));
    rtsiSetStepSizePtr(&helicopter1_M->solverInfo,
                       &helicopter1_M->Timing.stepSize0);
    rtsiSetdXPtr(&helicopter1_M->solverInfo, &helicopter1_M->derivs);
    rtsiSetContStatesPtr(&helicopter1_M->solverInfo, (real_T **)
                         &helicopter1_M->contStates);
    rtsiSetNumContStatesPtr(&helicopter1_M->solverInfo,
      &helicopter1_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&helicopter1_M->solverInfo,
      &helicopter1_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&helicopter1_M->solverInfo,
      &helicopter1_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&helicopter1_M->solverInfo,
      &helicopter1_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&helicopter1_M->solverInfo, (&rtmGetErrorStatus
      (helicopter1_M)));
    rtsiSetRTModelPtr(&helicopter1_M->solverInfo, helicopter1_M);
  }

  rtsiSetSimTimeStep(&helicopter1_M->solverInfo, MAJOR_TIME_STEP);
  helicopter1_M->intgData.f[0] = helicopter1_M->odeF[0];
  helicopter1_M->contStates = ((real_T *) &helicopter1_X);
  rtsiSetSolverData(&helicopter1_M->solverInfo, (void *)&helicopter1_M->intgData);
  rtsiSetSolverName(&helicopter1_M->solverInfo,"ode1");

  /* Initialize timing info */
  {
    int_T *mdlTsMap = helicopter1_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    helicopter1_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    helicopter1_M->Timing.sampleTimes = (&helicopter1_M->
      Timing.sampleTimesArray[0]);
    helicopter1_M->Timing.offsetTimes = (&helicopter1_M->
      Timing.offsetTimesArray[0]);

    /* task periods */
    helicopter1_M->Timing.sampleTimes[0] = (0.0);
    helicopter1_M->Timing.sampleTimes[1] = (0.002);

    /* task offsets */
    helicopter1_M->Timing.offsetTimes[0] = (0.0);
    helicopter1_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(helicopter1_M, &helicopter1_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = helicopter1_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    helicopter1_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(helicopter1_M, -1);
  helicopter1_M->Timing.stepSize0 = 0.002;
  helicopter1_M->Timing.stepSize1 = 0.002;
  helicopter1_M->solverInfoPtr = (&helicopter1_M->solverInfo);
  helicopter1_M->Timing.stepSize = (0.002);
  rtsiSetFixedStepSize(&helicopter1_M->solverInfo, 0.002);
  rtsiSetSolverMode(&helicopter1_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  helicopter1_M->blockIO = ((void *) &helicopter1_B);

  {
    helicopter1_B.TravelCounttorad = 0.0;
    helicopter1_B.Gain = 0.0;
    helicopter1_B.PitchCounttorad = 0.0;
    helicopter1_B.Gain_i = 0.0;
    helicopter1_B.ElevationCounttorad = 0.0;
    helicopter1_B.Sum = 0.0;
    helicopter1_B.FrontmotorSaturation = 0.0;
    helicopter1_B.BackmotorSaturation = 0.0;
    helicopter1_B.In1 = 0.0;
  }

  /* parameters */
  helicopter1_M->defaultParam = ((real_T *)&helicopter1_P);

  /* states (continuous) */
  {
    real_T *x = (real_T *) &helicopter1_X;
    helicopter1_M->contStates = (x);
    (void) memset((void *)&helicopter1_X, 0,
                  sizeof(X_helicopter1_T));
  }

  /* states (dwork) */
  helicopter1_M->dwork = ((void *) &helicopter1_DW);
  (void) memset((void *)&helicopter1_DW, 0,
                sizeof(DW_helicopter1_T));

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      helicopter1_DW.HILInitialize_AIMinimums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      helicopter1_DW.HILInitialize_AIMaximums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      helicopter1_DW.HILInitialize_AOMinimums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      helicopter1_DW.HILInitialize_AOMaximums[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      helicopter1_DW.HILInitialize_AOVoltages[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      helicopter1_DW.HILInitialize_FilterFrequency[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      helicopter1_DW.HILInitialize_POSortedFreqs[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      helicopter1_DW.HILInitialize_POValues[i] = 0.0;
    }
  }

  helicopter1_DW.HILWriteAnalog_Buffer[0] = 0.0;
  helicopter1_DW.HILWriteAnalog_Buffer[1] = 0.0;

  /* Initialize Sizes */
  helicopter1_M->Sizes.numContStates = (4);/* Number of continuous states */
  helicopter1_M->Sizes.numPeriodicContStates = (0);
                                      /* Number of periodic continuous states */
  helicopter1_M->Sizes.numY = (0);     /* Number of model outputs */
  helicopter1_M->Sizes.numU = (0);     /* Number of model inputs */
  helicopter1_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  helicopter1_M->Sizes.numSampTimes = (2);/* Number of sample times */
  helicopter1_M->Sizes.numBlocks = (46);/* Number of blocks */
  helicopter1_M->Sizes.numBlockIO = (9);/* Number of block outputs */
  helicopter1_M->Sizes.numBlockPrms = (144);/* Sum of parameter "widths" */
  return helicopter1_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/

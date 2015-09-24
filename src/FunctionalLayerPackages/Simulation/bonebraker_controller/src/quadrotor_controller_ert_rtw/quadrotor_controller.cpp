/*
 * File: quadrotor_controller.cpp
 *
 * Real-Time Workshop code generated for Simulink model quadrotor_controller.
 *
 * Model version                        : 1.61
 * Real-Time Workshop file version      : 7.6  (R2010b)  03-Aug-2010
 * Real-Time Workshop file generated on : Tue Jul 23 15:35:33 2013
 * TLC version                          : 7.6 (Jul 13 2010)
 * C/C++ source code generated on       : Tue Jul 23 15:35:33 2013
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Generic->Custom
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "quadrotor_controller.h"
#include "quadrotor_controller_private.h"
#include <stdio.h>
#include "quadrotor_controller_dt.h"

/* Block signals (auto storage) */
BlockIO_quadrotor_controller quadrotor_controller_B;

/* Block states (auto storage) */
D_Work_quadrotor_controller quadrotor_controller_DWork;

/* External inputs (root inport signals with auto storage) */
ExternalInputs_quadrotor_contro quadrotor_controller_U;

/* External outputs (root outports fed by signals with auto storage) */
ExternalOutputs_quadrotor_contr quadrotor_controller_Y;

/* Real-time model */
RT_MODEL_quadrotor_controller quadrotor_controller_M_;
RT_MODEL_quadrotor_controller *quadrotor_controller_M = &quadrotor_controller_M_;

/* Model step function */
void quadrotor_controller_step(void)
{
  int32_T i;

  {
    boolean_T rtmStopReq = FALSE;
    rtExtModePauseIfNeeded(quadrotor_controller_M->extModeInfo, 1, &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(quadrotor_controller_M, TRUE);
    }

    if (rtmGetStopRequested(quadrotor_controller_M) == TRUE) {
      rtmSetErrorStatus(quadrotor_controller_M, "Simulation finished");
      return;
    }
  }

  /* Outport: '<Root>/Force' incorporates:
   *  Constant: '<Root>/Constant'
   *  Constant: '<Root>/FORCE-Z'
   */
  quadrotor_controller_Y.Force[0] = quadrotor_controller_P.Constant_Value[0];
  quadrotor_controller_Y.Force[1] = quadrotor_controller_P.Constant_Value[1];
  quadrotor_controller_Y.Force[2] = quadrotor_controller_P.FORCEZ_Value;

  /* Outport: '<Root>/Torque' incorporates:
   *  Constant: '<Root>/Torque XYZ'
   */
  quadrotor_controller_Y.Torque[0] = quadrotor_controller_P.TorqueXYZ_Value[0];
  quadrotor_controller_Y.Torque[1] = quadrotor_controller_P.TorqueXYZ_Value[1];
  quadrotor_controller_Y.Torque[2] = quadrotor_controller_P.TorqueXYZ_Value[2];
  for (i = 0; i < 8; i++) {
    /* Gain: '<Root>/gp_6' incorporates:
     *  Inport: '<Root>/ARM Angles IN'
     */
    quadrotor_controller_B.gp_6[i] = quadrotor_controller_P.gp_6_Gain *
      quadrotor_controller_U.ARMAnglesIN[i];

    /* Outport: '<Root>/ARM Angles OUT' */
    quadrotor_controller_Y.ARMAnglesOUT[i] = quadrotor_controller_B.gp_6[i];
  }

  /* Gain: '<Root>/gp_7' incorporates:
   *  Inport: '<Root>/ARM Extended'
   */
  quadrotor_controller_B.gp_7 = quadrotor_controller_P.gp_7_Gain *
    quadrotor_controller_U.ARMExtended;

  /* Outport: '<Root>/ARM is Extended' */
  quadrotor_controller_Y.ARMisExtended = quadrotor_controller_B.gp_7;

  /* Gain: '<Root>/gp_9' incorporates:
   *  Inport: '<Root>/Position Command'
   */
  quadrotor_controller_B.gp_9[0] = quadrotor_controller_P.gp_9_Gain *
    quadrotor_controller_U.PositionCommand[0];
  quadrotor_controller_B.gp_9[1] = quadrotor_controller_P.gp_9_Gain *
    quadrotor_controller_U.PositionCommand[1];
  quadrotor_controller_B.gp_9[2] = quadrotor_controller_P.gp_9_Gain *
    quadrotor_controller_U.PositionCommand[2];

  /* Gain: '<Root>/gp_10' incorporates:
   *  Inport: '<Root>/Yaw Command'
   */
  quadrotor_controller_B.gp_10 = quadrotor_controller_P.gp_10_Gain *
    quadrotor_controller_U.YawCommand;

  /* Gain: '<Root>/gp_11' incorporates:
   *  Inport: '<Root>/Velocity Command'
   */
  quadrotor_controller_B.gp_11 = quadrotor_controller_P.gp_11_Gain *
    quadrotor_controller_U.VelocityCommand;

  /* Gain: '<Root>/gp_' incorporates:
   *  Inport: '<Root>/Position'
   */
  quadrotor_controller_B.gp_[0] = quadrotor_controller_P.gp_Gain *
    quadrotor_controller_U.Position[0];
  quadrotor_controller_B.gp_[1] = quadrotor_controller_P.gp_Gain *
    quadrotor_controller_U.Position[1];
  quadrotor_controller_B.gp_[2] = quadrotor_controller_P.gp_Gain *
    quadrotor_controller_U.Position[2];

  /* Gain: '<Root>/gp_1' incorporates:
   *  Inport: '<Root>/Attitude'
   */
  quadrotor_controller_B.gp_1[0] = quadrotor_controller_P.gp_1_Gain *
    quadrotor_controller_U.Attitude[0];
  quadrotor_controller_B.gp_1[1] = quadrotor_controller_P.gp_1_Gain *
    quadrotor_controller_U.Attitude[1];
  quadrotor_controller_B.gp_1[2] = quadrotor_controller_P.gp_1_Gain *
    quadrotor_controller_U.Attitude[2];

  /* Gain: '<Root>/gp_2' incorporates:
   *  Inport: '<Root>/Lineal Velocity'
   */
  quadrotor_controller_B.gp_2[0] = quadrotor_controller_P.gp_2_Gain *
    quadrotor_controller_U.LinealVelocity[0];
  quadrotor_controller_B.gp_2[1] = quadrotor_controller_P.gp_2_Gain *
    quadrotor_controller_U.LinealVelocity[1];
  quadrotor_controller_B.gp_2[2] = quadrotor_controller_P.gp_2_Gain *
    quadrotor_controller_U.LinealVelocity[2];

  /* Gain: '<Root>/gp_3' incorporates:
   *  Inport: '<Root>/Angular Velocity'
   */
  quadrotor_controller_B.gp_3[0] = quadrotor_controller_P.gp_3_Gain *
    quadrotor_controller_U.AngularVelocity[0];
  quadrotor_controller_B.gp_3[1] = quadrotor_controller_P.gp_3_Gain *
    quadrotor_controller_U.AngularVelocity[1];
  quadrotor_controller_B.gp_3[2] = quadrotor_controller_P.gp_3_Gain *
    quadrotor_controller_U.AngularVelocity[2];

  /* Gain: '<Root>/gp_8' incorporates:
   *  Inport: '<Root>/ARM Angles State IN'
   */
  for (i = 0; i < 8; i++) {
    quadrotor_controller_B.gp_8[i] = quadrotor_controller_P.gp_8_Gain *
      quadrotor_controller_U.ARMAnglesStateIN[i];
  }

  /* Gain: '<Root>/gp_13' incorporates:
   *  Inport: '<Root>/Joystick'
   */
  quadrotor_controller_B.gp_13[0] = quadrotor_controller_P.gp_13_Gain *
    quadrotor_controller_U.Joystick[0];
  quadrotor_controller_B.gp_13[1] = quadrotor_controller_P.gp_13_Gain *
    quadrotor_controller_U.Joystick[1];
  quadrotor_controller_B.gp_13[2] = quadrotor_controller_P.gp_13_Gain *
    quadrotor_controller_U.Joystick[2];
  quadrotor_controller_B.gp_13[3] = quadrotor_controller_P.gp_13_Gain *
    quadrotor_controller_U.Joystick[3];

  /* external mode */
  {
    boolean_T rtmStopReq = FALSE;
    rtExtModeOneStep(quadrotor_controller_M->extModeInfo, 1, &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(quadrotor_controller_M, TRUE);
    }
  }

  rtExtModeUploadCheckTrigger(1);

  {                                    /* Sample time: [0.01s, 0.0s] */
    rtExtModeUpload(0, quadrotor_controller_M->Timing.taskTime0);
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.01s, 0.0s] */
    if ((rtmGetTFinal(quadrotor_controller_M)!=-1) &&
        !((rtmGetTFinal(quadrotor_controller_M)-
           quadrotor_controller_M->Timing.taskTime0) >
          quadrotor_controller_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(quadrotor_controller_M, "Simulation finished");
    }

    if (rtmGetStopRequested(quadrotor_controller_M)) {
      rtmSetErrorStatus(quadrotor_controller_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  quadrotor_controller_M->Timing.taskTime0 =
    (++quadrotor_controller_M->Timing.clockTick0) *
    quadrotor_controller_M->Timing.stepSize0;
}

/* Model initialize function */
void quadrotor_controller_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)quadrotor_controller_M, 0,
                sizeof(RT_MODEL_quadrotor_controller));
  rtmSetTFinal(quadrotor_controller_M, -1);
  quadrotor_controller_M->Timing.stepSize0 = 0.01;

  /* external mode info */
  quadrotor_controller_M->Sizes.checksums[0] = (1732151426U);
  quadrotor_controller_M->Sizes.checksums[1] = (2449297721U);
  quadrotor_controller_M->Sizes.checksums[2] = (3212352586U);
  quadrotor_controller_M->Sizes.checksums[3] = (1308915177U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    quadrotor_controller_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(quadrotor_controller_M->extModeInfo,
      &quadrotor_controller_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(quadrotor_controller_M->extModeInfo,
                        quadrotor_controller_M->Sizes.checksums);
    rteiSetTPtr(quadrotor_controller_M->extModeInfo, rtmGetTPtr
                (quadrotor_controller_M));
  }

  /* block I/O */
  (void) memset(((void *) &quadrotor_controller_B), 0,
                sizeof(BlockIO_quadrotor_controller));

  /* states (dwork) */
  (void) memset((void *)&quadrotor_controller_DWork, 0,
                sizeof(D_Work_quadrotor_controller));

  /* external inputs */
  (void) memset((void *)&quadrotor_controller_U, 0,
                sizeof(ExternalInputs_quadrotor_contro));

  /* external outputs */
  (void) memset((void *)&quadrotor_controller_Y, 0,
                sizeof(ExternalOutputs_quadrotor_contr));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    quadrotor_controller_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 14;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.B = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.P = &rtPTransTable;
  }

  /* External mode */
  rtERTExtModeSetTFinal(&rtmGetTFinal(quadrotor_controller_M));
  rtExtModeCheckInit(1);

  {
    boolean_T rtmStopReq = FALSE;
    rtExtModeWaitForStartPkt(quadrotor_controller_M->extModeInfo, 1, &rtmStopReq);
    if (rtmStopReq) {
      rtmSetStopRequested(quadrotor_controller_M, TRUE);
    }
  }

  (void)printf("\n** starting the model **\n");
}

/* Model terminate function */
void quadrotor_controller_terminate(void)
{
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */

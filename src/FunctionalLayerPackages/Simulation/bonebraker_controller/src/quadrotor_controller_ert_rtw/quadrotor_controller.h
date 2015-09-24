/*
 * File: quadrotor_controller.h
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

#ifndef RTW_HEADER_quadrotor_controller_h_
#define RTW_HEADER_quadrotor_controller_h_
#ifndef quadrotor_controller_COMMON_INCLUDES_
# define quadrotor_controller_COMMON_INCLUDES_
#include <float.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "dt_info.h"
#include "ext_work.h"
#endif                                 /* quadrotor_controller_COMMON_INCLUDES_ */

#include "quadrotor_controller_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T gp_6[8];                      /* '<Root>/gp_6' */
  real_T gp_7;                         /* '<Root>/gp_7' */
  real_T gp_9[3];                      /* '<Root>/gp_9' */
  real_T gp_10;                        /* '<Root>/gp_10' */
  real_T gp_11;                        /* '<Root>/gp_11' */
  real_T gp_[3];                       /* '<Root>/gp_' */
  real_T gp_1[3];                      /* '<Root>/gp_1' */
  real_T gp_2[3];                      /* '<Root>/gp_2' */
  real_T gp_3[3];                      /* '<Root>/gp_3' */
  real_T gp_8[8];                      /* '<Root>/gp_8' */
  real_T gp_13[4];                     /* '<Root>/gp_13' */
} BlockIO_quadrotor_controller;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  struct {
    void *LoggedData;
  } ScopeARMAnglesIN_PWORK;            /* '<Root>/Scope ARM Angles IN' */

  struct {
    void *LoggedData;
  } ScopeARMExtendedIN_PWORK;          /* '<Root>/Scope ARM Extended IN' */

  struct {
    void *LoggedData;
  } ScopePositionCommand_PWORK;        /* '<Root>/Scope Position Command' */

  struct {
    void *LoggedData;
  } ScopeYawCommand_PWORK;             /* '<Root>/Scope Yaw Command' */

  struct {
    void *LoggedData;
  } ScopeVelocityCommand_PWORK;        /* '<Root>/Scope Velocity Command' */

  struct {
    void *LoggedData;
  } ScopePosition_PWORK;               /* '<Root>/Scope Position' */

  struct {
    void *LoggedData;
  } ScopeAttitude_PWORK;               /* '<Root>/Scope Attitude' */

  struct {
    void *LoggedData;
  } ScopeLinealVelocity_PWORK;         /* '<Root>/Scope Lineal Velocity' */

  struct {
    void *LoggedData;
  } ScopeAngularVelocity_PWORK;        /* '<Root>/Scope Angular Velocity' */

  struct {
    void *LoggedData;
  } ScopeARMAnglesStateIN_PWORK;       /* '<Root>/Scope ARM Angles State IN' */

  struct {
    void *LoggedData;
  } ScopeJoystick_PWORK;               /* '<Root>/Scope Joystick' */
} D_Work_quadrotor_controller;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T PositionCommand[3];           /* '<Root>/Position Command' */
  real_T YawCommand;                   /* '<Root>/Yaw Command' */
  real_T VelocityCommand;              /* '<Root>/Velocity Command' */
  real_T Position[3];                  /* '<Root>/Position' */
  real_T Attitude[3];                  /* '<Root>/Attitude' */
  real_T LinealVelocity[3];            /* '<Root>/Lineal Velocity' */
  real_T AngularVelocity[3];           /* '<Root>/Angular Velocity' */
  real_T ARMAnglesIN[8];               /* '<Root>/ARM Angles IN' */
  real_T ARMExtended;                  /* '<Root>/ARM Extended' */
  real_T ARMAnglesStateIN[8];          /* '<Root>/ARM Angles State IN' */
  real_T Joystick[4];                  /* '<Root>/Joystick' */
} ExternalInputs_quadrotor_contro;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T Force[3];                     /* '<Root>/Force' */
  real_T Torque[3];                    /* '<Root>/Torque' */
  real_T ARMAnglesOUT[8];              /* '<Root>/ARM Angles OUT' */
  real_T ARMisExtended;                /* '<Root>/ARM is Extended' */
} ExternalOutputs_quadrotor_contr;

/* Parameters (auto storage) */
struct Parameters_quadrotor_controller_ {
  real_T Constant_Value[2];            /* Expression: [0,0]
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T FORCEZ_Value;                 /* Expression: 0
                                        * Referenced by: '<Root>/FORCE-Z'
                                        */
  real_T TorqueXYZ_Value[3];           /* Expression: [0,0,0]
                                        * Referenced by: '<Root>/Torque XYZ'
                                        */
  real_T gp_6_Gain;                    /* Expression: 1
                                        * Referenced by: '<Root>/gp_6'
                                        */
  real_T gp_7_Gain;                    /* Expression: 1
                                        * Referenced by: '<Root>/gp_7'
                                        */
  real_T gp_9_Gain;                    /* Expression: 1
                                        * Referenced by: '<Root>/gp_9'
                                        */
  real_T gp_10_Gain;                   /* Expression: 1
                                        * Referenced by: '<Root>/gp_10'
                                        */
  real_T gp_11_Gain;                   /* Expression: 1
                                        * Referenced by: '<Root>/gp_11'
                                        */
  real_T gp_Gain;                      /* Expression: 1
                                        * Referenced by: '<Root>/gp_'
                                        */
  real_T gp_1_Gain;                    /* Expression: 1
                                        * Referenced by: '<Root>/gp_1'
                                        */
  real_T gp_2_Gain;                    /* Expression: 1
                                        * Referenced by: '<Root>/gp_2'
                                        */
  real_T gp_3_Gain;                    /* Expression: 1
                                        * Referenced by: '<Root>/gp_3'
                                        */
  real_T gp_8_Gain;                    /* Expression: 1
                                        * Referenced by: '<Root>/gp_8'
                                        */
  real_T gp_13_Gain;                   /* Expression: 1
                                        * Referenced by: '<Root>/gp_13'
                                        */
};

/* Real-time Model Data Structure */
struct RT_MODEL_quadrotor_controller {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T checksums[4];
  } Sizes;

  /*
   * SpecialInfo:
   * The following substructure contains special information
   * related to other components that are dependent on RTW.
   */
  struct {
    const void *mappingInfo;
  } SpecialInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (auto storage) */
extern Parameters_quadrotor_controller quadrotor_controller_P;

/* Block signals (auto storage) */
extern BlockIO_quadrotor_controller quadrotor_controller_B;

/* Block states (auto storage) */
extern D_Work_quadrotor_controller quadrotor_controller_DWork;

/* External inputs (root inport signals with auto storage) */
extern ExternalInputs_quadrotor_contro quadrotor_controller_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExternalOutputs_quadrotor_contr quadrotor_controller_Y;

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void quadrotor_controller_initialize(void);
  extern void quadrotor_controller_step(void);
  extern void quadrotor_controller_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern struct RT_MODEL_quadrotor_controller *quadrotor_controller_M;

#ifdef __cplusplus

}
#endif

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
 * '<Root>' : quadrotor_controller
 */
#endif                                 /* RTW_HEADER_quadrotor_controller_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */

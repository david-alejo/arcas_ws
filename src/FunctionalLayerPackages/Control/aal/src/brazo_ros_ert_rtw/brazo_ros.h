/*
 * File: brazo_ros.h
 *
 * Real-Time Workshop code generated for Simulink model brazo_ros.
 *
 * Model version                        : 1.28
 * Real-Time Workshop file version      : 7.6  (R2010b)  03-Aug-2010
 * Real-Time Workshop file generated on : Mon Jun 24 11:16:39 2013
 * TLC version                          : 7.6 (Jul 13 2010)
 * C/C++ source code generated on       : Mon Jun 24 11:16:40 2013
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Generic->32-bit x86 compatible
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_brazo_ros_h_
#define RTW_HEADER_brazo_ros_h_
#ifndef brazo_ros_COMMON_INCLUDES_
# define brazo_ros_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_defines.h"
#include "rt_pow_snf.h"
#endif                                 /* brazo_ros_COMMON_INCLUDES_ */

#include "brazo_ros_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T q1_1;                         /* '<Root>/Embedded MATLAB Function' */
  real_T q2_1;                         /* '<Root>/Embedded MATLAB Function' */
  real_T q3_1;                         /* '<Root>/Embedded MATLAB Function' */
  real_T q4_1;                         /* '<Root>/Embedded MATLAB Function' */
  real_T q5_1;                         /* '<Root>/Embedded MATLAB Function' */
} D_Work_brazo_ros;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T P_X;                          /* '<Root>/P_X' */
  real_T P_X1;                         /* '<Root>/P_X1' */
  real_T P_X2;                         /* '<Root>/P_X2' */
} ExternalInputs_brazo_ros;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T Q_1;                          /* '<Root>/Q_1' */
  real_T Q_2;                          /* '<Root>/Q_2' */
  real_T Q_3;                          /* '<Root>/Q_3' */
  real_T Q_4;                          /* '<Root>/Q_4' */
  real_T Q_5;                          /* '<Root>/Q_5' */
} ExternalOutputs_brazo_ros;

/* Parameters (auto storage) */
struct Parameters_brazo_ros_ {
  real_T Constant1_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T Constant2_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant2'
                                        */
  real_T Constant3_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant3'
                                        */
};

/* Real-time Model Data Structure */
struct RT_MODEL_brazo_ros {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern Parameters_brazo_ros brazo_ros_P;

/* Block states (auto storage) */
extern D_Work_brazo_ros brazo_ros_DWork;

/* External inputs (root inport signals with auto storage) */
extern ExternalInputs_brazo_ros brazo_ros_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExternalOutputs_brazo_ros brazo_ros_Y;

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void brazo_ros_initialize(void);
  extern void brazo_ros_step(void);
  extern void brazo_ros_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern struct RT_MODEL_brazo_ros *brazo_ros_M;

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
 * '<Root>' : brazo_ros
 * '<S1>'   : brazo_ros/Embedded MATLAB Function
 */
#endif                                 /* RTW_HEADER_brazo_ros_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */

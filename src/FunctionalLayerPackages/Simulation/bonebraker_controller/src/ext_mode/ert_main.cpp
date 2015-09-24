/*
 * File: ert_main.cpp
 *
 * Real-Time Workshop code generated for Simulink model ControlCoche.
 *
 * Model version                        : 1.53
 * Real-Time Workshop file version      : 7.6.1  (R2010bSP1)  28-Jan-2011
 * Real-Time Workshop file generated on : Mon May 27 18:36:42 2013
 * TLC version                          : 7.6 (Jul 13 2010)
 * C/C++ source code generated on       : Mon May 27 18:36:42 2013
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM 7
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include <stdio.h>                     /* This ert_main.c example uses printf/fflush */
#include "ControlCoche.h"              /* Model's header file */
#include "rtwtypes.h"                  /* MathWorks types */
#include "ext_work.h"                  /* External mode header file */

/*
 * Associating rt_OneStep with a real-time clock or interrupt service routine
 * is what makes the generated code "real-time".  The function rt_OneStep is
 * always associated with the base rate of the model.  Subrates are managed
 * by the base rate from inside the generated code.  Enabling/disabling
 * interrupts and floating point context switches are target specific.  This
 * example code indicates where these should take place relative to executing
 * the generated code step function.  Overrun behavior should be tailored to
 * your application needs.  This example simply sets an error status in the
 * real-time model and returns from rt_OneStep.
 */
void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag = 0;

  /* Disable interrupts here */

  /* Check for overrun */
  if (OverrunFlag) {
    rtmSetErrorStatus(ControlCoche_M, "Overrun");
    return;
  }

  OverrunFlag = TRUE;

  /* Save FPU context here (if necessary) */
  /* Re-enable timer or interrupt here */
  /* Set model inputs here */

  /* Step the model */
  ControlCoche_step();

  /* Get model outputs here */

  /* Indicate task complete */
  OverrunFlag = FALSE;

  /* Disable interrupts here */
  /* Restore FPU context here (if necessary) */
  /* Enable interrupts here */
  rtExtModeCheckEndTrigger();
}

/*
 * The example "main" function illustrates what is required by your
 * application code to initialize, execute, and terminate the generated code.
 * Attaching rt_OneStep to a real-time clock is target specific.  This example
 * illustates how you do this relative to initializing the model.
 */
int_T main(int_T argc, const char_T *argv[]);
int_T main(int_T argc, const char_T *argv[])
{
  /* External mode */
  rtERTExtModeParseArgs(argc, argv);

  /* Initialize model */
  ControlCoche_initialize();

  /* The External Mode option selected; therefore,
   *  simulating the model step behavior (in non real-time).
   */
  while ((rtmGetErrorStatus(ControlCoche_M) == (NULL)) && !rtmGetStopRequested
         (ControlCoche_M)) {
    rt_OneStep();
  }

  /* External mode */
  rtExtModeShutdown(1);

  /* Disable rt_OneStep() here */

  /* Terminate model */
  ControlCoche_terminate();
  return 0;
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */

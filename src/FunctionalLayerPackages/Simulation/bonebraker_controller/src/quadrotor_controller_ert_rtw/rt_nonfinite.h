/*
 * File: rt_nonfinite.h
 *
 * Real-Time Workshop code generated for Simulink model quadrotor_controller.
 *
 * Model version                        : 1.60
 * Real-Time Workshop file version      : 7.6  (R2010b)  03-Aug-2010
 * Real-Time Workshop file generated on : Tue Jul 23 11:15:57 2013
 * TLC version                          : 7.6 (Jul 13 2010)
 * C/C++ source code generated on       : Tue Jul 23 11:15:58 2013
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Generic->Custom
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_rt_nonfinite_h_
#define RTW_HEADER_rt_nonfinite_h_
#include <stddef.h>
#include "rtwtypes.h"
#ifdef __cplusplus

extern "C" {

#endif

  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  extern void rt_InitInfAndNaN(size_t realSize);
  extern boolean_T rtIsInf(real_T value);
  extern boolean_T rtIsInfF(real32_T value);
  extern boolean_T rtIsNaN(real_T value);
  extern boolean_T rtIsNaNF(real32_T value);
  typedef struct {
    struct {
      uint32_T wordH;
      uint32_T wordL;
    } words;
  } BigEndianIEEEDouble;

  typedef struct {
    struct {
      uint32_T wordL;
      uint32_T wordH;
    } words;
  } LittleEndianIEEEDouble;

  typedef struct {
    union {
      real32_T wordLreal;
      uint32_T wordLuint;
    } wordL;
  } IEEESingle;

#ifdef __cplusplus

}                                      /* extern "C" */
#endif
#endif                                 /* RTW_HEADER_rt_nonfinite_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */

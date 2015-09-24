/*
 * File: rt_nonfinite.cpp
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

/*
 * Abstract:
 *      Real-Time Workshop function to intialize non-finites,
 *      (Inf, NaN and -Inf).
 */
#include "rt_nonfinite.h"
#include "rtGetNaN.h"
#include "rtGetInf.h"

extern "C" {
  real_T rtInf;
  real_T rtMinusInf;
  real_T rtNaN;
  real32_T rtInfF;
  real32_T rtMinusInfF;
  real32_T rtNaNF;
}
  extern "C"
{
  /*
   * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
   * generated code. NaN is initialized as non-signaling. Assumes IEEE.
   */
  void rt_InitInfAndNaN(size_t realSize)
  {
    (void) (realSize);
    rtNaN = rtGetNaN();
    rtNaNF = rtGetNaNF();
    rtInf = rtGetInf();
    rtInfF = rtGetInfF();
    rtMinusInf = rtGetMinusInf();
    rtMinusInfF = rtGetMinusInfF();
  }

  /* Test if value is infinite */
  boolean_T rtIsInf(real_T value)
  {
    return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
  }

  /* Test if single-precision value is infinite */
  boolean_T rtIsInfF(real32_T value)
  {
    return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
  }

  /* Test if value is not a number */
  boolean_T rtIsNaN(real_T value)
  {

#if defined(_MSC_VER) && (_MSC_VER <= 1200)

    /* For MSVC 6.0, use a compiler specific comparison function */
    return (boolean_T)(_isnan(value)? 1U:0U);

#else

    return (boolean_T)((value!=value) ? 1U : 0U);

#endif

  }

  /* Test if single-precision value is not a number */
  boolean_T rtIsNaNF(real32_T value)
  {

#if defined(_MSC_VER) && (_MSC_VER <= 1200)

    /* For MSVC 6.0, use a compiler specific comparison function */
    return (boolean_T)(_isnan((real_T)value)? 1U:0U);

#else

    return (boolean_T)(((value!=value) ? 1U : 0U));

#endif

  }
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */

/*
 * File: rtGetNaN.cpp
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
 *      Real-Time Workshop function to intialize non-finite, NaN
 */
#include "rtGetNaN.h"
#define NumBitsPerChar                 8U

extern "C" {
  /*
   * Initialize rtNaN needed by the generated code.
   * NaN is initialized as non-signaling. Assumes IEEE.
   */
  real_T rtGetNaN(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T nan = 0.0;
    if (bitsPerReal == 32U) {
      nan = rtGetNaNF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF80000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      nan = tmpVal.fltVal;
    }

    return nan;
  }

  /*
   * Initialize rtNaNF needed by the generated code.
   * NaN is initialized as non-signaling. Assumes IEEE.
   */
  real32_T rtGetNaNF(void)
  {
    IEEESingle nanF;
    nanF.wordL.wordLuint = 0xFFC00000U;
    return nanF.wordL.wordLreal;
  }
}
/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */

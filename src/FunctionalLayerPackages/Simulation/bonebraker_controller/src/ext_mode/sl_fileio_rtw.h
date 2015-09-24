#ifdef SUPPORTS_PRAGMA_ONCE
# pragma once
#endif
#ifndef sl_fileio_rtw_h
#define sl_fileio_rtw_h
/*
 *
 * Copyright 2008-2010 The MathWorks, Inc.
 *
 * This header is the intrace to the sl_fileio module. It is used by the
 * RTW-RSim and Rapid Accelerartor targets.
 *
 * $Revision: 1.1.6.8 $
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

extern const char *rtwH5LoggingCreateInstance(
    const char     *fileName, 
    int            errorXMLMode,
    const char     *objName,
    const char     *sigName,
    const int       dTypeId,
    const int       nDims,
    const int      *dims,
    const int       isComplex,
    const int       decimation,
    const char     *formatStr,
    const int       isFixedPoint,
    const int       dataTypeMode,
    const int       isSigned,
    const int       wordLength,
    const double    slopeAdjustmentFactor,
    const int       fixedExponent,
    const double    bias,
    void          **outH5logging);

extern const char *rtwH5LoggingDestroyInstance(
    int  errorXMLMode,
    void *h5logging);

extern const char *rtwH5LoggingWrite(
    int          errorXMLMode,
    void         *h5logging,
    const double  t,
    const void   *u);

extern const char *rtwSetMcosObjName(
    const char *fileName,
    int        errorXMLMode,
    const char *objName);

extern const char *rtwMATFileLoaderCreateInstance(
    const char  *fileName,
    int          errorXMLMode,
    const char  *extrapolationBeforeFirstDataPointStr,
    const char  *interpolationWithinTimeRangeStr,
    const char  *extrapolationAfterLastDataPointStr,
    const unsigned char *ground,
    int          hardwareinfoDevType,
    int          hardwareInfoBitsPerCIntegers0,
    int          hardwareInfoBitsPerCIntegers1,
    int          hardwareInfoBitsPerCIntegers2,
    int          hardwareInfoBitsPerCIntegers3,
    int          hardwareinfoDivRounding,
    int          fxpDiagnosticOverflow,
    const char  *fxpBlockPath,
    void        **outMATFileLoader);

extern const char *rtwMATFileLoaderDestroyInstance(
    int  errorXMLMode,
    void *pMATFileLoader);

extern const char *rtwMATFileLoaderGetOutput(
    int   errorXMLMode,
    void *pMATFileLoader,
    const double t,
    void **outOutputValue);

extern const char *rtwMATFileLoaderCheckMATFileConsistency(
    void           *pMATFileLoader,
    const char     *MATFileName,
    int             errorXMLMode,
    const char     *expDTypeName,
    const int       expNDims,
    const int      *expDims,
    const int       expComplexity,
    const int       isFixedPoint,
    const int       dataTypeMode,
    const int       isSigned,
    const int       wordLength,
    const double    slopeAdjustmentFactor,
    const int       fixedExponent,
    const double    bias,
    const int       expEnumNStrings,
    const char    **expEnumStrings,
    const int      *expEnumValues);

#ifdef __cplusplus
}
#endif

#endif /* sl_fileio_rtw_h */

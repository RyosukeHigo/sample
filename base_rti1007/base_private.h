/*
 * base_private.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "base".
 *
 * Model version              : 1.294
 * Simulink Coder version : 8.10 (R2016a) 10-Feb-2016
 * C source code generated on : Thu Jan 03 20:45:22 2019
 *
 * Target selection: rti1007.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_base_private_h_
#define RTW_HEADER_base_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "zero_crossing_types.h"

extern void TaskSet(SimStruct *rts);
extern void handEnc2JntAng(SimStruct *rts);
extern void rtiethernetdecode(SimStruct *rts);
extern void handTraj(SimStruct *rts);
extern void handJntAngLimit(SimStruct *rts);
extern void handCtrl(SimStruct *rts);
extern void handJntTrq2DA(SimStruct *rts);
extern void handDALimit(SimStruct *rts);
extern void rtiethernetencode(SimStruct *rts);
extern void XYZstageTraj(SimStruct *rts);
extern void XYZstageAngLimit(SimStruct *rts);
extern void XYZstageEnc2JntAng(SimStruct *rts);
extern void XYZstageCtrl(SimStruct *rts);
extern void XYZstageTrq2DA(SimStruct *rts);
extern void XYZstageDALimit(SimStruct *rts);
extern void SystemSet(SimStruct *rts);

#endif                                 /* RTW_HEADER_base_private_h_ */

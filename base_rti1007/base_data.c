/*
 * base_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "base".
 *
 * Model version              : 1.294
 * Simulink Coder version : 8.10 (R2016a) 10-Feb-2016
 * C source code generated on : Thu Nov 01 18:11:40 2018
 *
 * Target selection: rti1007.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "base.h"
#include "base_private.h"

/* Block parameters (auto storage) */
P_base_T base_P = {
  /*  Expression: [0 0 0 0 0 0 0 0 0 0 ]
   * Referenced by: '<S7>/dummy'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /*  Computed Parameter: ETHERNET_DECODE_BL2_P1_Size
   * Referenced by: '<S8>/ETHERNET_DECODE_BL2'
   */
  { 1.0, 1.0 },
  2.0,                                 /* Expression: byteordering
                                        * Referenced by: '<S8>/ETHERNET_DECODE_BL2'
                                        */

  /*  Computed Parameter: ETHERNET_DECODE_BL2_P2_Size
   * Referenced by: '<S8>/ETHERNET_DECODE_BL2'
   */
  { 1.0, 1.0 },
  0.0,                                 /* Expression: offset
                                        * Referenced by: '<S8>/ETHERNET_DECODE_BL2'
                                        */

  /*  Computed Parameter: ETHERNET_DECODE_BL2_P3_Size
   * Referenced by: '<S8>/ETHERNET_DECODE_BL2'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: auto_offset
                                        * Referenced by: '<S8>/ETHERNET_DECODE_BL2'
                                        */

  /*  Computed Parameter: ETHERNET_DECODE_BL2_P4_Size
   * Referenced by: '<S8>/ETHERNET_DECODE_BL2'
   */
  { 1.0, 1.0 },
  9.0,                                 /* Expression: datatype
                                        * Referenced by: '<S8>/ETHERNET_DECODE_BL2'
                                        */

  /*  Computed Parameter: ETHERNET_DECODE_BL2_P5_Size
   * Referenced by: '<S8>/ETHERNET_DECODE_BL2'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: one_datatype
                                        * Referenced by: '<S8>/ETHERNET_DECODE_BL2'
                                        */

  /*  Computed Parameter: ETHERNET_DECODE_BL2_P6_Size
   * Referenced by: '<S8>/ETHERNET_DECODE_BL2'
   */
  { 1.0, 1.0 },
  -1.0,                                /* Expression: sampletime
                                        * Referenced by: '<S8>/ETHERNET_DECODE_BL2'
                                        */

  /*  Computed Parameter: ETHERNET_ENCODE_BL1_P1_Size
   * Referenced by: '<S9>/ETHERNET_ENCODE_BL1'
   */
  { 1.0, 1.0 },
  2.0,                                 /* Expression: byteordering
                                        * Referenced by: '<S9>/ETHERNET_ENCODE_BL1'
                                        */

  /*  Computed Parameter: ETHERNET_ENCODE_BL1_P2_Size
   * Referenced by: '<S9>/ETHERNET_ENCODE_BL1'
   */
  { 1.0, 1.0 },
  0.0,                                 /* Expression: offset
                                        * Referenced by: '<S9>/ETHERNET_ENCODE_BL1'
                                        */

  /*  Computed Parameter: ETHERNET_ENCODE_BL1_P3_Size
   * Referenced by: '<S9>/ETHERNET_ENCODE_BL1'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: auto_offset
                                        * Referenced by: '<S9>/ETHERNET_ENCODE_BL1'
                                        */

  /*  Computed Parameter: ETHERNET_ENCODE_BL1_P4_Size
   * Referenced by: '<S9>/ETHERNET_ENCODE_BL1'
   */
  { 1.0, 1.0 },
  9.0,                                 /* Expression: datatype
                                        * Referenced by: '<S9>/ETHERNET_ENCODE_BL1'
                                        */

  /*  Computed Parameter: ETHERNET_ENCODE_BL1_P5_Size
   * Referenced by: '<S9>/ETHERNET_ENCODE_BL1'
   */
  { 1.0, 1.0 },
  1.0,                                 /* Expression: one_datatype
                                        * Referenced by: '<S9>/ETHERNET_ENCODE_BL1'
                                        */

  /*  Computed Parameter: ETHERNET_ENCODE_BL1_P6_Size
   * Referenced by: '<S9>/ETHERNET_ENCODE_BL1'
   */
  { 1.0, 1.0 },
  -1.0,                                /* Expression: sampletime
                                        * Referenced by: '<S9>/ETHERNET_ENCODE_BL1'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<S5>/S-Function2'
                                        */
  1,                                   /* Expression: boolean(1)
                                        * Referenced by: '<S8>/Receive Switcher'
                                        */
  1                                    /* Expression: boolean(1)
                                        * Referenced by: '<S9>/Send Switcher'
                                        */
};

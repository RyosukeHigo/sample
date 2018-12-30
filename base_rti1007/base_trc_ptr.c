/***************************************************************************

   Source file base_trc_ptr.c:

   Definition of function that initializes the global TRC pointers

   RTI1007 7.6 (02-May-2016)
   Sun Dec 30 22:18:13 2018

   Copyright 2018, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

/* Include header file. */
#include "base_trc_ptr.h"
#include "base.h"
#include "base_private.h"

/* Compiler options to turn off optimization. */
#if !defined(DS_OPTIMIZE_INIT_TRC_POINTERS)
#ifdef _MCCPPC

#pragma options -nOt -nOr -nOi -nOx

#endif

#ifdef __GNUC__

#pragma GCC optimize ("O0")

#endif

#ifdef _MSC_VER

#pragma optimize ("", off)

#endif
#endif

/* Definition of Global pointers to data type transitions (for TRC-file access) */
volatile real_T *p_0_base_real_T_0 = NULL;
volatile int32_T *p_0_base_int32_T_1 = NULL;
volatile uint32_T *p_0_base_uint32_T_2 = NULL;
volatile uint8_T *p_0_base_uint8_T_3 = NULL;
volatile real_T *p_1_base_real_T_0 = NULL;
volatile boolean_T *p_1_base_boolean_T_1 = NULL;
volatile real_T *p_2_base_real_T_0 = NULL;
volatile int_T *p_2_base_int_T_2 = NULL;

/*
 *  Declare the functions, that initially assign TRC pointers
 */
static void rti_init_trc_pointers_0(void);

/* Global pointers to data type transitions are separated in different functions to avoid overloading */
static void rti_init_trc_pointers_0(void)
{
  p_0_base_real_T_0 = &base_B.TaskSetting;
  p_0_base_int32_T_1 = &base_B.SystemSetting;
  p_0_base_uint32_T_2 = &base_B.SFunction1[0];
  p_0_base_uint8_T_3 = &base_B.SFunction1_o1_f[0];
  p_1_base_real_T_0 = &base_P.dummy_Value[0];
  p_1_base_boolean_T_1 = &base_P.ReceiveSwitcher_Value;
  p_2_base_real_T_0 = &base_DW.SFunction1_RWORK.RX_DROPPED_FRAMES[0];
  p_2_base_int_T_2 = &base_DW.SystemSetting_IWORK;
}

void base_rti_init_trc_pointers(void)
{
  rti_init_trc_pointers_0();
}

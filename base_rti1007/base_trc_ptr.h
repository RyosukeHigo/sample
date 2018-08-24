  /*********************** dSPACE target specific file *************************

   Header file base_trc_ptr.h:

   Declaration of function that initializes the global TRC pointers

   RTI1007 7.6 (02-May-2016)
   Fri Aug 03 14:28:41 2018

   Copyright 2018, dSPACE GmbH. All rights reserved.

  *****************************************************************************/
  #ifndef RTI_HEADER_base_trc_ptr_h_
  #define RTI_HEADER_base_trc_ptr_h_
  /* Include the model header file. */
  #include "base.h"
  #include "base_private.h"

  #ifdef EXTERN_C
  #undef EXTERN_C
  #endif

  #ifdef __cplusplus
  #define EXTERN_C                       extern "C"
  #else
  #define EXTERN_C                       extern
  #endif

  /*
   *  Declare the global TRC pointers
   */
              EXTERN_C volatile  real_T *p_0_base_real_T_0;
              EXTERN_C volatile  int32_T *p_0_base_int32_T_1;
              EXTERN_C volatile  uint32_T *p_0_base_uint32_T_2;
              EXTERN_C volatile  uint8_T *p_0_base_uint8_T_3;
              EXTERN_C volatile  real_T *p_1_base_real_T_0;
              EXTERN_C volatile  boolean_T *p_1_base_boolean_T_1;
              EXTERN_C volatile  real_T *p_2_base_real_T_0;
              EXTERN_C volatile  int_T *p_2_base_int_T_2;

  /*
   *  Declare the general function for TRC pointer initialization
   */
  EXTERN_C void base_rti_init_trc_pointers(void);
   #endif                       /* RTI_HEADER_base_trc_ptr_h_ */
/*
 * base.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "base".
 *
 * Model version              : 1.294
 * Simulink Coder version : 8.10 (R2016a) 10-Feb-2016
 * C source code generated on : Wed Dec 26 13:47:48 2018
 *
 * Target selection: rti1007.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "base_trc_ptr.h"
#include "base.h"
#include "base_private.h"

const real_T base_RGND = 0.0;          /* real_T ground */

/* Block signals (auto storage) */
B_base_T base_B;

/* Block states (auto storage) */
DW_base_T base_DW;

/* Previous zero-crossings (trigger) states */
PrevZCX_base_T base_PrevZCX;

/* Real-time model */
RT_MODEL_base_T base_M_;
RT_MODEL_base_T *const base_M = &base_M_;

/* Model output function */
static void base_output(void)
{
  ZCEventType zcEvent;

  /* Level2 S-Function Block: '<Root>/System Setting' (SystemSet) */
  {
    SimStruct *rts = base_M->childSfunctions[15];
    sfcnOutputs(rts, 0);
  }

  /* Outputs for Enabled SubSystem: '<Root>/Application' incorporates:
   *  EnablePort: '<S1>/Enable'
   */
  if (base_B.SystemSetting > 0) {
    /* Level2 S-Function Block: '<S1>/Task Setting' (TaskSet) */
    {
      SimStruct *rts = base_M->childSfunctions[0];
      sfcnOutputs(rts, 0);
    }

    /* Level2 S-Function Block: '<S7>/handEnc2JntAng' (handEnc2JntAng) */
    {
      SimStruct *rts = base_M->childSfunctions[1];
      sfcnOutputs(rts, 0);
    }

    /* Constant: '<S7>/dummy' */
    memcpy(&base_B.dummy[0], &base_P.dummy_Value[0], 10U * sizeof(real_T));

    /* S-Function (rti_commonblock): '<S12>/S-Function1' */
    /* This comment workarounds a code generation problem */
    {
      /* --- base/Application/UDP Receive/ETHERNET_UDP_RX_BL1: ==> Socket ID = (0) --- */
      /* dSPACE I/O Board DSETHERNET #1 Unit:RXUDP Group:RXUDP */

      /* fixed frame size mode is adjusted */

      /* variable declarations */
      DsSSockAddrIn remoteAddr;
      UInt32 addrLen = sizeof(remoteAddr);
      Int32 realLen, tmpStatus;

      /* set remote IP and port initially to 0 within the socket structure */
      remoteAddr.sin_addr.s_addr = (UInt32) 0;
      remoteAddr.sin_port = (UInt16) 0;
      remoteAddr.sin_family = DSIOETH_AF_INET;

      /* whether block is enabled */
      if (base_P.ReceiveSwitcher_Value == 0) {
        /* block is disabled */
        /**/

        /* set output Status to state 1 indicating a disabled RX block */
        tmpStatus = 1;
      } else {
        /* block is enabled */
        /**/

        /* receive data from a socket and obtain the address of the sender */
        realLen =
          DsIoEth_recvfrom( DSIOETH_CONNECTION_ID_0,
                           (uint8_T *) &base_B.SFunction1_o1_f[0],
                           112U,
                           DSIOETH_FLAG_NONE,
                           (DsSSockAddr *) &remoteAddr,
                           &addrLen
                           );
        if (realLen > 0) {
          /* set output Status to state 0 indicating successfully received data */
          tmpStatus = 0;

          /* increment output Received Frames  */
          base_B.SFunction1_o3 = (real_T) (base_B.SFunction1_o3 + 1);
        } else {
          /* realLen -1: No data received due to empty rx buffer *
           * realLen  0: Connection socket not ready (closed)    */
          tmpStatus = (realLen == -1) ? 2 : 3;
        }
      }                                // if (Inport_Enable == 0)

      /* assign receive status to the related outport */
      base_B.SFunction1_o2_h = (uint32_T) tmpStatus;
    }

    /* Level2 S-Function Block: '<S8>/ETHERNET_DECODE_BL2' (rtiethernetdecode) */
    {
      SimStruct *rts = base_M->childSfunctions[2];
      sfcnOutputs(rts, 0);
    }

    /* Level2 S-Function Block: '<S7>/handTraj' (handTraj) */
    {
      SimStruct *rts = base_M->childSfunctions[3];
      sfcnOutputs(rts, 0);
    }

    /* Level2 S-Function Block: '<S7>/handJntAngLimit' (handJntAngLimit) */
    {
      SimStruct *rts = base_M->childSfunctions[4];
      sfcnOutputs(rts, 0);
    }

    /* Level2 S-Function Block: '<S6>/handCtrl' (handCtrl) */
    {
      SimStruct *rts = base_M->childSfunctions[5];
      sfcnOutputs(rts, 0);
    }

    /* Level2 S-Function Block: '<S6>/handJntTrq2DA' (handJntTrq2DA) */
    {
      SimStruct *rts = base_M->childSfunctions[6];
      sfcnOutputs(rts, 0);
    }

    /* Level2 S-Function Block: '<S6>/handDALimit' (handDALimit) */
    {
      SimStruct *rts = base_M->childSfunctions[7];
      sfcnOutputs(rts, 0);
    }

    /* Level2 S-Function Block: '<S9>/ETHERNET_ENCODE_BL1' (rtiethernetencode) */
    {
      SimStruct *rts = base_M->childSfunctions[8];
      sfcnOutputs(rts, 0);
    }

    /* S-Function (rti_commonblock): '<S13>/S-Function1' */
    /* This comment workarounds a code generation problem */
    {
      /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP Group:TXUDP */

      /* variable declarations */
      UInt8 txData[96U];
      UInt8 *pData = (UInt8*) &txData[0];
      UInt8 txRemoteIp[4];
      UInt16 txRemotePort;
      DsSSockAddrIn txRemoteAddr ;
      UInt32 addrLen = sizeof(txRemoteAddr);
      Int32 bytesToSendValid, realLen, tmpStatus;

      /* whether block is enabled */
      if (base_P.SendSwitcher_Value == 0) {
        /* block is disabled */
        /**/

        /* set output Status to state 1 indicating a disabled TX block */
        tmpStatus = 1;
      } else {
        /* block is enabled */
        /**/

        /* fixed frame size mode is adjusted */
        /* parameter FrameSize determines the amount of data to be send */
        bytesToSendValid = 96U;

        /* set a specific remote IP within the socket structure (to send data to) */
        txRemoteIp[0] = 10U;
        txRemoteIp[1] = 1U;
        txRemoteIp[2] = 196U;
        txRemoteIp[3] = 178U;

        /* static specification of the remote port by block parameter (to send data to) */
        txRemotePort = 52001U;

        /* set remote IP and port to the socket structure */
        txRemoteAddr.sin_addr.s_addr = *((UInt32*)txRemoteIp);
        txRemoteAddr.sin_port = DsIoEth_htons(txRemotePort);
        txRemoteAddr.sin_family = DSIOETH_AF_INET;

        {
          int_T i1;
          const uint8_T *u0 = &base_B.ETHERNET_ENCODE_BL1[0];
          for (i1=0; i1 < 96; i1++) {
            *pData++ = u0[i1];
          }
        }

        /* sends data over a socket to a specific target address */
        realLen =
          DsIoEth_sendto( DSIOETH_CONNECTION_ID_0,
                         (uint8_T *) txData,
                         bytesToSendValid,
                         DSIOETH_FLAG_NONE,
                         (DsSSockAddr *) &txRemoteAddr,
                         addrLen
                         );
        if (realLen > 0) {
          /* increment output Send Frames  */
          base_B.SFunction1_o2 = (real_T) (base_B.SFunction1_o2 + 1);

          /* set output Status to 0 indicating successfully sent data */
          tmpStatus = 0;
        } else {
          /* realLen -1: No data sent due to filled tx buffer *
           * realLen  0: Connection socket not ready (closed) */
          tmpStatus = (realLen == -1) ? 2 : 3;
        }
      }                                // if (Inport_Enable == 0)

      /* assign send status to the related outport */
      base_B.SFunction1_o1 = (uint32_T) tmpStatus;
    }

    /* Level2 S-Function Block: '<S11>/XYZstageTraj' (XYZstageTraj) */
    {
      SimStruct *rts = base_M->childSfunctions[9];
      sfcnOutputs(rts, 0);
    }

    /* Level2 S-Function Block: '<S11>/XYZstageJntAngLimit' (XYZstageAngLimit) */
    {
      SimStruct *rts = base_M->childSfunctions[10];
      sfcnOutputs(rts, 0);
    }

    /* Level2 S-Function Block: '<S11>/XYZstageEnc2JntAng' (XYZstageEnc2JntAng) */
    {
      SimStruct *rts = base_M->childSfunctions[11];
      sfcnOutputs(rts, 0);
    }

    /* Level2 S-Function Block: '<S10>/XYZstageCtrl' (XYZstageCtrl) */
    {
      SimStruct *rts = base_M->childSfunctions[12];
      sfcnOutputs(rts, 0);
    }

    /* Level2 S-Function Block: '<S10>/XYZstageTrq2DA' (XYZstageTrq2DA) */
    {
      SimStruct *rts = base_M->childSfunctions[13];
      sfcnOutputs(rts, 0);
    }

    /* Level2 S-Function Block: '<S10>/XYZstageDALimit' (XYZstageDALimit) */
    {
      SimStruct *rts = base_M->childSfunctions[14];
      sfcnOutputs(rts, 0);
    }
  }

  /* End of Outputs for SubSystem: '<Root>/Application' */

  /* Outputs for Triggered SubSystem: '<Root>/simState SET' incorporates:
   *  TriggerPort: '<S5>/Trigger'
   */
  zcEvent = rt_I32ZCFcn(FALLING_ZERO_CROSSING,&base_PrevZCX.simStateSET_Trig_ZCE,
                        (base_B.SystemSetting));
  if (zcEvent != NO_ZCEVENT) {
    /* S-Function (rti_commonblock): '<S5>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* dSPACE S-Function Block: <S5>/S-Function1 (simState SET) */
    simState = STOP;
  }

  /* End of Outputs for SubSystem: '<Root>/simState SET' */

  /* S-Function (rti_commonblock): '<S2>/S-Function1' */
  /* This comment workarounds a code generation problem */

  /* --- base/ETHERNET_SETUP_BL1 --- */
  /* dSPACE I/O Board DSETHERNET #1 Unit:SETUP Group:SETUP */
  {
    /* returns the link state of the ethernet connection. Number of available connectors: uint32_T) */
    base_B.SFunction1[0] = (uint32_T) DsIoEth_getLinkState((UInt32) 0);
    base_B.SFunction1[1] = (uint32_T) DsIoEth_getLinkState((UInt32) 1);
  }

  /* S-Function (rti_commonblock): '<S3>/S-Function1' */
  /* This comment workarounds a code generation problem */

  /* --- base/ETHERNET_UDP_SETUP_BL1: ==> Socket ID = (0) --- */
  {
    /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP Group:SETUPUDP */

    /* gets the port state (open or closed) of the specified UDP socket */
    if ((DsIoEth_getPortState(DSIOETH_CONNECTION_ID_0) == DSIOETH_PORT_CLOSED))
    {
      /* Opens a non-blocking UDP connection. A Socket must be created before, using    *
       * DsIoEth_create(). It also flushs the related rx socket receive queue to ensure *
       * not to accept any data from a previously adjusted communication                */
      DsIoEth_open(DSIOETH_CONNECTION_ID_0);
    }

    /* reads out the pending management event (internal event queue must *
     * be always read, to prevent event queue overrun)                   */
    DsIoEth_getMgmtEvent(DSIOETH_CONNECTION_ID_0);
  }
}

/* Model update function */
static void base_update(void)
{
  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++base_M->Timing.clockTick0)) {
    ++base_M->Timing.clockTickH0;
  }

  base_M->Timing.t[0] = base_M->Timing.clockTick0 * base_M->Timing.stepSize0 +
    base_M->Timing.clockTickH0 * base_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
static void base_initialize(void)
{
  /* Level2 S-Function Block: '<Root>/System Setting' (SystemSet) */
  {
    SimStruct *rts = base_M->childSfunctions[15];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for Enabled SubSystem: '<Root>/Application' */
  /* Level2 S-Function Block: '<S1>/Task Setting' (TaskSet) */
  {
    SimStruct *rts = base_M->childSfunctions[0];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for Constant: '<S7>/dummy' */
  memcpy(&base_B.dummy[0], &base_P.dummy_Value[0], 10U * sizeof(real_T));

  /* Level2 S-Function Block: '<S8>/ETHERNET_DECODE_BL2' (rtiethernetdecode) */
  {
    SimStruct *rts = base_M->childSfunctions[2];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Level2 S-Function Block: '<S7>/handTraj' (handTraj) */
  {
    SimStruct *rts = base_M->childSfunctions[3];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Level2 S-Function Block: '<S6>/handCtrl' (handCtrl) */
  {
    SimStruct *rts = base_M->childSfunctions[5];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Level2 S-Function Block: '<S9>/ETHERNET_ENCODE_BL1' (rtiethernetencode) */
  {
    SimStruct *rts = base_M->childSfunctions[8];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Level2 S-Function Block: '<S11>/XYZstageTraj' (XYZstageTraj) */
  {
    SimStruct *rts = base_M->childSfunctions[9];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Level2 S-Function Block: '<S10>/XYZstageCtrl' (XYZstageCtrl) */
  {
    SimStruct *rts = base_M->childSfunctions[12];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Level2 S-Function Block: '<S10>/XYZstageDALimit' (XYZstageDALimit) */
  {
    SimStruct *rts = base_M->childSfunctions[14];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* End of Start for SubSystem: '<Root>/Application' */
  base_PrevZCX.simStateSET_Trig_ZCE = UNINITIALIZED_ZCSIG;
}

/* Model terminate function */
static void base_terminate(void)
{
  /* Level2 S-Function Block: '<Root>/System Setting' (SystemSet) */
  {
    SimStruct *rts = base_M->childSfunctions[15];
    sfcnTerminate(rts);
  }

  /* Terminate for Enabled SubSystem: '<Root>/Application' */

  /* Level2 S-Function Block: '<S1>/Task Setting' (TaskSet) */
  {
    SimStruct *rts = base_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S7>/handEnc2JntAng' (handEnc2JntAng) */
  {
    SimStruct *rts = base_M->childSfunctions[1];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S8>/ETHERNET_DECODE_BL2' (rtiethernetdecode) */
  {
    SimStruct *rts = base_M->childSfunctions[2];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S7>/handTraj' (handTraj) */
  {
    SimStruct *rts = base_M->childSfunctions[3];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S7>/handJntAngLimit' (handJntAngLimit) */
  {
    SimStruct *rts = base_M->childSfunctions[4];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S6>/handCtrl' (handCtrl) */
  {
    SimStruct *rts = base_M->childSfunctions[5];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S6>/handJntTrq2DA' (handJntTrq2DA) */
  {
    SimStruct *rts = base_M->childSfunctions[6];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S6>/handDALimit' (handDALimit) */
  {
    SimStruct *rts = base_M->childSfunctions[7];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S9>/ETHERNET_ENCODE_BL1' (rtiethernetencode) */
  {
    SimStruct *rts = base_M->childSfunctions[8];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S11>/XYZstageTraj' (XYZstageTraj) */
  {
    SimStruct *rts = base_M->childSfunctions[9];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S11>/XYZstageJntAngLimit' (XYZstageAngLimit) */
  {
    SimStruct *rts = base_M->childSfunctions[10];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S11>/XYZstageEnc2JntAng' (XYZstageEnc2JntAng) */
  {
    SimStruct *rts = base_M->childSfunctions[11];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S10>/XYZstageCtrl' (XYZstageCtrl) */
  {
    SimStruct *rts = base_M->childSfunctions[12];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S10>/XYZstageTrq2DA' (XYZstageTrq2DA) */
  {
    SimStruct *rts = base_M->childSfunctions[13];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<S10>/XYZstageDALimit' (XYZstageDALimit) */
  {
    SimStruct *rts = base_M->childSfunctions[14];
    sfcnTerminate(rts);
  }

  /* End of Terminate for SubSystem: '<Root>/Application' */

  /* Terminate for S-Function (rti_commonblock): '<S3>/S-Function1' */
  {
    /* --- base/ETHERNET_UDP_SETUP_BL1: ==> Socket ID = (0) --- */
    /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP Group:SETUPUDP */

    /* close a socket connection
     * After closing, the corresponding socket resource is still occupied.   *
     * If a connection has been closed and should be re-used for a different *
     * connection, or if a connection should be re-established it must be    *
     * re-opened using DsIoEth_open() again.                                 */
    DsIoEth_close(DSIOETH_CONNECTION_ID_0);
  }
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  base_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  base_update();
  UNUSED_PARAMETER(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  base_initialize();
}

void MdlTerminate(void)
{
  base_terminate();
}

/* Registration function */
RT_MODEL_base_T *base(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)base_M, 0,
                sizeof(RT_MODEL_base_T));
  rtsiSetSolverName(&base_M->solverInfo,"FixedStepDiscrete");
  base_M->solverInfoPtr = (&base_M->solverInfo);

  /* Initialize timing info */
  {
    int_T *mdlTsMap = base_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    base_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    base_M->Timing.sampleTimes = (&base_M->Timing.sampleTimesArray[0]);
    base_M->Timing.offsetTimes = (&base_M->Timing.offsetTimesArray[0]);

    /* task periods */
    base_M->Timing.sampleTimes[0] = (0.001);

    /* task offsets */
    base_M->Timing.offsetTimes[0] = (0.0);
  }

  rtmSetTPtr(base_M, &base_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = base_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    base_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(base_M, -1);
  base_M->Timing.stepSize0 = 0.001;
  base_M->solverInfoPtr = (&base_M->solverInfo);
  base_M->Timing.stepSize = (0.001);
  rtsiSetFixedStepSize(&base_M->solverInfo, 0.001);
  rtsiSetSolverMode(&base_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  base_M->ModelData.blockIO = ((void *) &base_B);
  (void) memset(((void *) &base_B), 0,
                sizeof(B_base_T));

  {
    int32_T i;
    for (i = 0; i < 11; i++) {
      base_B.handEnc2JntAng_n[i] = 0.0;
    }

    for (i = 0; i < 10; i++) {
      base_B.dummy[i] = 0.0;
    }

    for (i = 0; i < 14; i++) {
      base_B.ETHERNET_DECODE_BL2[i] = 0.0;
    }

    for (i = 0; i < 11; i++) {
      base_B.handTraj_o1[i] = 0.0;
    }

    for (i = 0; i < 11; i++) {
      base_B.handJntAngLimit_i[i] = 0.0;
    }

    for (i = 0; i < 11; i++) {
      base_B.handCtrl_d[i] = 0.0;
    }

    for (i = 0; i < 11; i++) {
      base_B.handJntTrq2DA_d[i] = 0.0;
    }

    for (i = 0; i < 11; i++) {
      base_B.handDALimit_e[i] = 0.0;
    }

    base_B.TaskSetting = 0.0;
    base_B.SFunction1_o3 = 0.0;
    base_B.handTraj_o2 = 0.0;
    base_B.SFunction1_o2 = 0.0;
    base_B.XYZstageTraj_o1[0] = 0.0;
    base_B.XYZstageTraj_o1[1] = 0.0;
    base_B.XYZstageTraj_o1[2] = 0.0;
    base_B.XYZstageTraj_o2[0] = 0.0;
    base_B.XYZstageTraj_o2[1] = 0.0;
    base_B.XYZstageTraj_o2[2] = 0.0;
    base_B.XYZstageJntAngLimit[0] = 0.0;
    base_B.XYZstageJntAngLimit[1] = 0.0;
    base_B.XYZstageJntAngLimit[2] = 0.0;
    base_B.XYZstageEnc2JntAng_a[0] = 0.0;
    base_B.XYZstageEnc2JntAng_a[1] = 0.0;
    base_B.XYZstageEnc2JntAng_a[2] = 0.0;
    base_B.XYZstageCtrl_h[0] = 0.0;
    base_B.XYZstageCtrl_h[1] = 0.0;
    base_B.XYZstageCtrl_h[2] = 0.0;
    base_B.XYZstageTrq2DA_p[0] = 0.0;
    base_B.XYZstageTrq2DA_p[1] = 0.0;
    base_B.XYZstageTrq2DA_p[2] = 0.0;
    base_B.XYZstageDALimit_j[0] = 0.0;
    base_B.XYZstageDALimit_j[1] = 0.0;
    base_B.XYZstageDALimit_j[2] = 0.0;
  }

  /* parameters */
  base_M->ModelData.defaultParam = ((real_T *)&base_P);

  /* states (dwork) */
  base_M->ModelData.dwork = ((void *) &base_DW);
  (void) memset((void *)&base_DW, 0,
                sizeof(DW_base_T));
  base_DW.SFunction1_RWORK.RX_DROPPED_FRAMES[0] = 0.0;
  base_DW.SFunction1_RWORK.RX_DROPPED_FRAMES[1] = 0.0;
  base_DW.TaskSetting_RWORK = 0.0;
  base_DW.SFunction1_RWORK_i.RECEIVED_FRAMES = 0.0;

  {
    int32_T i;
    for (i = 0; i < 11; i++) {
      base_DW.handTraj_RWORK[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 11; i++) {
      base_DW.handCtrl_RWORK[i] = 0.0;
    }
  }

  base_DW.handDALimit_RWORK = 0.0;
  base_DW.SFunction1_RWORK_k.SENT_FRAMES = 0.0;
  base_DW.XYZstageTraj_RWORK[0] = 0.0;
  base_DW.XYZstageTraj_RWORK[1] = 0.0;
  base_DW.XYZstageTraj_RWORK[2] = 0.0;
  base_DW.XYZstageCtrl_RWORK[0] = 0.0;
  base_DW.XYZstageCtrl_RWORK[1] = 0.0;
  base_DW.XYZstageCtrl_RWORK[2] = 0.0;
  base_DW.XYZstageDALimit_RWORK = 0.0;

  /* child S-Function registration */
  {
    RTWSfcnInfo *sfcnInfo = &base_M->NonInlinedSFcns.sfcnInfo;
    base_M->sfcnInfo = (sfcnInfo);
    rtssSetErrorStatusPtr(sfcnInfo, (&rtmGetErrorStatus(base_M)));
    rtssSetNumRootSampTimesPtr(sfcnInfo, &base_M->Sizes.numSampTimes);
    base_M->NonInlinedSFcns.taskTimePtrs[0] = &(rtmGetTPtr(base_M)[0]);
    rtssSetTPtrPtr(sfcnInfo,base_M->NonInlinedSFcns.taskTimePtrs);
    rtssSetTStartPtr(sfcnInfo, &rtmGetTStart(base_M));
    rtssSetTFinalPtr(sfcnInfo, &rtmGetTFinal(base_M));
    rtssSetTimeOfLastOutputPtr(sfcnInfo, &rtmGetTimeOfLastOutput(base_M));
    rtssSetStepSizePtr(sfcnInfo, &base_M->Timing.stepSize);
    rtssSetStopRequestedPtr(sfcnInfo, &rtmGetStopRequested(base_M));
    rtssSetDerivCacheNeedsResetPtr(sfcnInfo,
      &base_M->ModelData.derivCacheNeedsReset);
    rtssSetZCCacheNeedsResetPtr(sfcnInfo, &base_M->ModelData.zCCacheNeedsReset);
    rtssSetBlkStateChangePtr(sfcnInfo, &base_M->ModelData.blkStateChange);
    rtssSetSampleHitsPtr(sfcnInfo, &base_M->Timing.sampleHits);
    rtssSetPerTaskSampleHitsPtr(sfcnInfo, &base_M->Timing.perTaskSampleHits);
    rtssSetSimModePtr(sfcnInfo, &base_M->simMode);
    rtssSetSolverInfoPtr(sfcnInfo, &base_M->solverInfoPtr);
  }

  base_M->Sizes.numSFcns = (16);

  /* register each child */
  {
    (void) memset((void *)&base_M->NonInlinedSFcns.childSFunctions[0], 0,
                  16*sizeof(SimStruct));
    base_M->childSfunctions = (&base_M->NonInlinedSFcns.childSFunctionPtrs[0]);

    {
      int_T i;
      for (i = 0; i < 16; i++) {
        base_M->childSfunctions[i] = (&base_M->NonInlinedSFcns.childSFunctions[i]);
      }
    }

    /* Level2 S-Function Block: base/<S1>/Task Setting (TaskSet) */
    {
      SimStruct *rts = base_M->childSfunctions[0];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn0.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn0.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn0.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[0]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[0]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[0]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[0]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[0]);
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn0.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 1);
          ssSetOutputPortSignal(rts, 0, ((real_T *) &base_B.TaskSetting));
        }
      }

      /* path info */
      ssSetModelName(rts, "Task Setting");
      ssSetPath(rts, "base/Application/Task Setting");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetRWork(rts, (real_T *) &base_DW.TaskSetting_RWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &base_M->NonInlinedSFcns.Sfcn0.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &base_M->NonInlinedSFcns.Sfcn0.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &base_DW.TaskSetting_RWORK);
      }

      /* registration */
      TaskSet(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
    }

    /* Level2 S-Function Block: base/<S7>/handEnc2JntAng (handEnc2JntAng) */
    {
      SimStruct *rts = base_M->childSfunctions[1];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn1.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn1.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn1.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[1]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[1]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[1]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[1]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[1]);
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn1.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 11);
          ssSetOutputPortSignal(rts, 0, ((real_T *) base_B.handEnc2JntAng_n));
        }
      }

      /* path info */
      ssSetModelName(rts, "handEnc2JntAng");
      ssSetPath(rts, "base/Application/Hand Trajectory/handEnc2JntAng");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* registration */
      handEnc2JntAng(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
    }

    /* Level2 S-Function Block: base/<S8>/ETHERNET_DECODE_BL2 (rtiethernetdecode) */
    {
      SimStruct *rts = base_M->childSfunctions[2];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn2.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn2.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn2.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[2]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[2]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[2]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[2]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[2]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts, &base_M->
          NonInlinedSFcns.Sfcn2.inputPortInfo[0]);

        /* port 0 */
        {
          uint8_T const **sfcnUPtrs = (uint8_T const **)
            &base_M->NonInlinedSFcns.Sfcn2.UPtrs0;

          {
            int_T i1;
            const uint8_T *u0 = base_B.SFunction1_o1_f;
            for (i1=0; i1 < 112; i1++) {
              sfcnUPtrs[i1] = &u0[i1];
            }
          }

          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 112);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn2.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 14);
          ssSetOutputPortSignal(rts, 0, ((real_T *) base_B.ETHERNET_DECODE_BL2));
        }
      }

      /* path info */
      ssSetModelName(rts, "ETHERNET_DECODE_BL2");
      ssSetPath(rts, "base/Application/UDP Receive/ETHERNET_DECODE_BL2");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &base_M->NonInlinedSFcns.Sfcn2.params;
        ssSetSFcnParamsCount(rts, 6);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)base_P.ETHERNET_DECODE_BL2_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)base_P.ETHERNET_DECODE_BL2_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)base_P.ETHERNET_DECODE_BL2_P3_Size);
        ssSetSFcnParam(rts, 3, (mxArray*)base_P.ETHERNET_DECODE_BL2_P4_Size);
        ssSetSFcnParam(rts, 4, (mxArray*)base_P.ETHERNET_DECODE_BL2_P5_Size);
        ssSetSFcnParam(rts, 5, (mxArray*)base_P.ETHERNET_DECODE_BL2_P6_Size);
      }

      /* work vectors */
      ssSetIWork(rts, (int_T *) &base_DW.ETHERNET_DECODE_BL2_IWORK);
      ssSetPWork(rts, (void **) &base_DW.ETHERNET_DECODE_BL2_PWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &base_M->NonInlinedSFcns.Sfcn2.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &base_M->NonInlinedSFcns.Sfcn2.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 2);

        /* IWORK */
        ssSetDWorkWidth(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &base_DW.ETHERNET_DECODE_BL2_IWORK);

        /* PWORK */
        ssSetDWorkWidth(rts, 1, 1);
        ssSetDWorkDataType(rts, 1,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &base_DW.ETHERNET_DECODE_BL2_PWORK);
      }

      /* registration */
      rtiethernetdecode(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetInputPortWidth(rts, 0, 112);
      ssSetInputPortDataType(rts, 0, SS_UINT8);
      ssSetInputPortComplexSignal(rts, 0, 0);
      ssSetInputPortFrameData(rts, 0, 0);
      ssSetOutputPortWidth(rts, 0, 14);
      ssSetOutputPortDataType(rts, 0, SS_DOUBLE);
      ssSetOutputPortComplexSignal(rts, 0, 0);
      ssSetOutputPortFrameData(rts, 0, 0);
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: base/<S7>/handTraj (handTraj) */
    {
      SimStruct *rts = base_M->childSfunctions[3];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn3.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn3.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn3.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[3]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[3]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[3]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[3]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[3]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 4);
        ssSetPortInfoForInputs(rts, &base_M->
          NonInlinedSFcns.Sfcn3.inputPortInfo[0]);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn3.UPtrs0;
          sfcnUPtrs[0] = &base_B.TaskSetting;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }

        /* port 1 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn3.UPtrs1;

          {
            int_T i1;
            const real_T *u1 = base_B.handEnc2JntAng_n;
            for (i1=0; i1 < 11; i1++) {
              sfcnUPtrs[i1] = &u1[i1];
            }
          }

          ssSetInputPortSignalPtrs(rts, 1, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 1, 1);
          ssSetInputPortWidth(rts, 1, 11);
        }

        /* port 2 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn3.UPtrs2;

          {
            int_T i1;
            const real_T *u2 = base_B.dummy;
            for (i1=0; i1 < 10; i1++) {
              sfcnUPtrs[i1] = &u2[i1];
            }
          }

          ssSetInputPortSignalPtrs(rts, 2, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 2, 1);
          ssSetInputPortWidth(rts, 2, 10);
        }

        /* port 3 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn3.UPtrs3;

          {
            int_T i1;
            const real_T *u3 = base_B.ETHERNET_DECODE_BL2;
            for (i1=0; i1 < 14; i1++) {
              sfcnUPtrs[i1] = &u3[i1];
            }
          }

          ssSetInputPortSignalPtrs(rts, 3, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 3, 1);
          ssSetInputPortWidth(rts, 3, 14);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn3.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 2);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 11);
          ssSetOutputPortSignal(rts, 0, ((real_T *) base_B.handTraj_o1));
        }

        /* port 1 */
        {
          _ssSetOutputPortNumDimensions(rts, 1, 1);
          ssSetOutputPortWidth(rts, 1, 1);
          ssSetOutputPortSignal(rts, 1, ((real_T *) &base_B.handTraj_o2));
        }
      }

      /* path info */
      ssSetModelName(rts, "handTraj");
      ssSetPath(rts, "base/Application/Hand Trajectory/handTraj");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetRWork(rts, (real_T *) &base_DW.handTraj_RWORK[0]);
      ssSetIWork(rts, (int_T *) &base_DW.handTraj_IWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &base_M->NonInlinedSFcns.Sfcn3.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &base_M->NonInlinedSFcns.Sfcn3.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 2);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 11);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &base_DW.handTraj_RWORK[0]);

        /* IWORK */
        ssSetDWorkWidth(rts, 1, 1);
        ssSetDWorkDataType(rts, 1,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &base_DW.handTraj_IWORK);
      }

      /* registration */
      handTraj(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetInputPortConnected(rts, 2, 1);
      _ssSetInputPortConnected(rts, 3, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 0);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
      ssSetInputPortBufferDstPort(rts, 2, -1);
      ssSetInputPortBufferDstPort(rts, 3, -1);
    }

    /* Level2 S-Function Block: base/<S7>/handJntAngLimit (handJntAngLimit) */
    {
      SimStruct *rts = base_M->childSfunctions[4];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn4.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn4.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn4.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[4]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[4]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[4]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[4]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[4]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts, &base_M->
          NonInlinedSFcns.Sfcn4.inputPortInfo[0]);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn4.UPtrs0;

          {
            int_T i1;
            const real_T *u0 = base_B.handTraj_o1;
            for (i1=0; i1 < 11; i1++) {
              sfcnUPtrs[i1] = &u0[i1];
            }
          }

          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 11);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn4.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 11);
          ssSetOutputPortSignal(rts, 0, ((real_T *) base_B.handJntAngLimit_i));
        }
      }

      /* path info */
      ssSetModelName(rts, "handJntAngLimit");
      ssSetPath(rts, "base/Application/Hand Trajectory/handJntAngLimit");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* registration */
      handJntAngLimit(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: base/<S6>/handCtrl (handCtrl) */
    {
      SimStruct *rts = base_M->childSfunctions[5];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn5.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn5.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn5.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[5]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[5]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[5]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[5]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[5]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 2);
        ssSetPortInfoForInputs(rts, &base_M->
          NonInlinedSFcns.Sfcn5.inputPortInfo[0]);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn5.UPtrs0;

          {
            int_T i1;
            const real_T *u0 = base_B.handJntAngLimit_i;
            for (i1=0; i1 < 11; i1++) {
              sfcnUPtrs[i1] = &u0[i1];
            }
          }

          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 11);
        }

        /* port 1 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn5.UPtrs1;

          {
            int_T i1;
            const real_T *u1 = base_B.handEnc2JntAng_n;
            for (i1=0; i1 < 11; i1++) {
              sfcnUPtrs[i1] = &u1[i1];
            }
          }

          ssSetInputPortSignalPtrs(rts, 1, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 1, 1);
          ssSetInputPortWidth(rts, 1, 11);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn5.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 11);
          ssSetOutputPortSignal(rts, 0, ((real_T *) base_B.handCtrl_d));
        }
      }

      /* path info */
      ssSetModelName(rts, "handCtrl");
      ssSetPath(rts, "base/Application/Hand Control/handCtrl");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetRWork(rts, (real_T *) &base_DW.handCtrl_RWORK[0]);
      ssSetIWork(rts, (int_T *) &base_DW.handCtrl_IWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &base_M->NonInlinedSFcns.Sfcn5.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &base_M->NonInlinedSFcns.Sfcn5.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 2);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 11);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &base_DW.handCtrl_RWORK[0]);

        /* IWORK */
        ssSetDWorkWidth(rts, 1, 1);
        ssSetDWorkDataType(rts, 1,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &base_DW.handCtrl_IWORK);
      }

      /* registration */
      handCtrl(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
    }

    /* Level2 S-Function Block: base/<S6>/handJntTrq2DA (handJntTrq2DA) */
    {
      SimStruct *rts = base_M->childSfunctions[6];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn6.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn6.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn6.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[6]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[6]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[6]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[6]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[6]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts, &base_M->
          NonInlinedSFcns.Sfcn6.inputPortInfo[0]);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn6.UPtrs0;

          {
            int_T i1;
            const real_T *u0 = base_B.handCtrl_d;
            for (i1=0; i1 < 11; i1++) {
              sfcnUPtrs[i1] = &u0[i1];
            }
          }

          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 11);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn6.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 11);
          ssSetOutputPortSignal(rts, 0, ((real_T *) base_B.handJntTrq2DA_d));
        }
      }

      /* path info */
      ssSetModelName(rts, "handJntTrq2DA");
      ssSetPath(rts, "base/Application/Hand Control/handJntTrq2DA");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* registration */
      handJntTrq2DA(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: base/<S6>/handDALimit (handDALimit) */
    {
      SimStruct *rts = base_M->childSfunctions[7];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn7.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn7.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn7.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[7]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[7]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[7]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[7]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[7]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts, &base_M->
          NonInlinedSFcns.Sfcn7.inputPortInfo[0]);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn7.UPtrs0;

          {
            int_T i1;
            const real_T *u0 = base_B.handJntTrq2DA_d;
            for (i1=0; i1 < 11; i1++) {
              sfcnUPtrs[i1] = &u0[i1];
            }
          }

          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 11);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn7.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 11);
          ssSetOutputPortSignal(rts, 0, ((real_T *) base_B.handDALimit_e));
        }
      }

      /* path info */
      ssSetModelName(rts, "handDALimit");
      ssSetPath(rts, "base/Application/Hand Control/handDALimit");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetRWork(rts, (real_T *) &base_DW.handDALimit_RWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &base_M->NonInlinedSFcns.Sfcn7.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &base_M->NonInlinedSFcns.Sfcn7.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &base_DW.handDALimit_RWORK);
      }

      /* registration */
      handDALimit(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: base/<S9>/ETHERNET_ENCODE_BL1 (rtiethernetencode) */
    {
      SimStruct *rts = base_M->childSfunctions[8];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn8.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn8.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn8.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[8]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[8]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[8]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[8]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[8]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts, &base_M->
          NonInlinedSFcns.Sfcn8.inputPortInfo[0]);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn8.UPtrs0;
          sfcnUPtrs[0] = &base_B.TaskSetting;

          {
            int_T i1;
            const real_T *u0 = &base_B.handEnc2JntAng_n[0];
            for (i1=0; i1 < 11; i1++) {
              sfcnUPtrs[i1+ 1] = &u0[i1];
            }
          }

          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 12);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn8.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 96);
          ssSetOutputPortSignal(rts, 0, ((uint8_T *) base_B.ETHERNET_ENCODE_BL1));
        }
      }

      /* path info */
      ssSetModelName(rts, "ETHERNET_ENCODE_BL1");
      ssSetPath(rts, "base/Application/UDP Send/ETHERNET_ENCODE_BL1");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &base_M->NonInlinedSFcns.Sfcn8.params;
        ssSetSFcnParamsCount(rts, 6);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)base_P.ETHERNET_ENCODE_BL1_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)base_P.ETHERNET_ENCODE_BL1_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)base_P.ETHERNET_ENCODE_BL1_P3_Size);
        ssSetSFcnParam(rts, 3, (mxArray*)base_P.ETHERNET_ENCODE_BL1_P4_Size);
        ssSetSFcnParam(rts, 4, (mxArray*)base_P.ETHERNET_ENCODE_BL1_P5_Size);
        ssSetSFcnParam(rts, 5, (mxArray*)base_P.ETHERNET_ENCODE_BL1_P6_Size);
      }

      /* work vectors */
      ssSetIWork(rts, (int_T *) &base_DW.ETHERNET_ENCODE_BL1_IWORK);
      ssSetPWork(rts, (void **) &base_DW.ETHERNET_ENCODE_BL1_PWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &base_M->NonInlinedSFcns.Sfcn8.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &base_M->NonInlinedSFcns.Sfcn8.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 2);

        /* IWORK */
        ssSetDWorkWidth(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &base_DW.ETHERNET_ENCODE_BL1_IWORK);

        /* PWORK */
        ssSetDWorkWidth(rts, 1, 1);
        ssSetDWorkDataType(rts, 1,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &base_DW.ETHERNET_ENCODE_BL1_PWORK);
      }

      /* registration */
      rtiethernetencode(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetInputPortWidth(rts, 0, 12);
      ssSetInputPortDataType(rts, 0, SS_DOUBLE);
      ssSetInputPortComplexSignal(rts, 0, 0);
      ssSetInputPortFrameData(rts, 0, 0);
      ssSetOutputPortWidth(rts, 0, 96);
      ssSetOutputPortDataType(rts, 0, SS_UINT8);
      ssSetOutputPortComplexSignal(rts, 0, 0);
      ssSetOutputPortFrameData(rts, 0, 0);
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: base/<S11>/XYZstageTraj (XYZstageTraj) */
    {
      SimStruct *rts = base_M->childSfunctions[9];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn9.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn9.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn9.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[9]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[9]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[9]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[9]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[9]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 4);
        ssSetPortInfoForInputs(rts, &base_M->
          NonInlinedSFcns.Sfcn9.inputPortInfo[0]);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn9.UPtrs0;
          sfcnUPtrs[0] = &base_B.TaskSetting;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }

        /* port 1 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn9.UPtrs1;
          sfcnUPtrs[0] = (real_T*)&base_RGND;
          sfcnUPtrs[1] = (real_T*)&base_RGND;
          sfcnUPtrs[2] = (real_T*)&base_RGND;
          sfcnUPtrs[3] = (real_T*)&base_RGND;
          ssSetInputPortSignalPtrs(rts, 1, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 1, 1);
          ssSetInputPortWidth(rts, 1, 4);
        }

        /* port 2 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn9.UPtrs2;

          {
            int_T i1;
            for (i1=0; i1 < 18; i1++) {
              sfcnUPtrs[i1] = (real_T*)&base_RGND;
            }
          }

          ssSetInputPortSignalPtrs(rts, 2, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 2, 1);
          ssSetInputPortWidth(rts, 2, 18);
        }

        /* port 3 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn9.UPtrs3;

          {
            int_T i1;
            const real_T *u3 = base_B.ETHERNET_DECODE_BL2;
            for (i1=0; i1 < 14; i1++) {
              sfcnUPtrs[i1] = &u3[i1];
            }
          }

          ssSetInputPortSignalPtrs(rts, 3, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 3, 1);
          ssSetInputPortWidth(rts, 3, 14);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn9.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 2);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 3);
          ssSetOutputPortSignal(rts, 0, ((real_T *) base_B.XYZstageTraj_o1));
        }

        /* port 1 */
        {
          _ssSetOutputPortNumDimensions(rts, 1, 1);
          ssSetOutputPortWidth(rts, 1, 3);
          ssSetOutputPortSignal(rts, 1, ((real_T *) base_B.XYZstageTraj_o2));
        }
      }

      /* path info */
      ssSetModelName(rts, "XYZstageTraj");
      ssSetPath(rts, "base/Application/XYZstage Trajectory/XYZstageTraj");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetRWork(rts, (real_T *) &base_DW.XYZstageTraj_RWORK[0]);
      ssSetIWork(rts, (int_T *) &base_DW.XYZstageTraj_IWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &base_M->NonInlinedSFcns.Sfcn9.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &base_M->NonInlinedSFcns.Sfcn9.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 2);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 3);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &base_DW.XYZstageTraj_RWORK[0]);

        /* IWORK */
        ssSetDWorkWidth(rts, 1, 1);
        ssSetDWorkDataType(rts, 1,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &base_DW.XYZstageTraj_IWORK);
      }

      /* registration */
      XYZstageTraj(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 0);
      _ssSetInputPortConnected(rts, 2, 0);
      _ssSetInputPortConnected(rts, 3, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 0);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
      ssSetInputPortBufferDstPort(rts, 2, -1);
      ssSetInputPortBufferDstPort(rts, 3, -1);
    }

    /* Level2 S-Function Block: base/<S11>/XYZstageJntAngLimit (XYZstageAngLimit) */
    {
      SimStruct *rts = base_M->childSfunctions[10];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn10.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn10.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn10.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[10]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[10]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[10]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[10]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[10]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &base_M->NonInlinedSFcns.Sfcn10.inputPortInfo[0]);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn10.UPtrs0;
          sfcnUPtrs[0] = base_B.XYZstageTraj_o1;
          sfcnUPtrs[1] = &base_B.XYZstageTraj_o1[1];
          sfcnUPtrs[2] = &base_B.XYZstageTraj_o1[2];
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 3);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn10.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 3);
          ssSetOutputPortSignal(rts, 0, ((real_T *) base_B.XYZstageJntAngLimit));
        }
      }

      /* path info */
      ssSetModelName(rts, "XYZstageJntAngLimit");
      ssSetPath(rts, "base/Application/XYZstage Trajectory/XYZstageJntAngLimit");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* registration */
      XYZstageAngLimit(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: base/<S11>/XYZstageEnc2JntAng (XYZstageEnc2JntAng) */
    {
      SimStruct *rts = base_M->childSfunctions[11];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn11.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn11.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn11.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[11]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[11]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[11]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[11]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[11]);
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn11.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 3);
          ssSetOutputPortSignal(rts, 0, ((real_T *) base_B.XYZstageEnc2JntAng_a));
        }
      }

      /* path info */
      ssSetModelName(rts, "XYZstageEnc2JntAng");
      ssSetPath(rts, "base/Application/XYZstage Trajectory/XYZstageEnc2JntAng");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* registration */
      XYZstageEnc2JntAng(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
    }

    /* Level2 S-Function Block: base/<S10>/XYZstageCtrl (XYZstageCtrl) */
    {
      SimStruct *rts = base_M->childSfunctions[12];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn12.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn12.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn12.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[12]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[12]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[12]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[12]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[12]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 2);
        ssSetPortInfoForInputs(rts,
          &base_M->NonInlinedSFcns.Sfcn12.inputPortInfo[0]);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn12.UPtrs0;
          sfcnUPtrs[0] = base_B.XYZstageJntAngLimit;
          sfcnUPtrs[1] = &base_B.XYZstageJntAngLimit[1];
          sfcnUPtrs[2] = &base_B.XYZstageJntAngLimit[2];
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 3);
        }

        /* port 1 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn12.UPtrs1;
          sfcnUPtrs[0] = base_B.XYZstageEnc2JntAng_a;
          sfcnUPtrs[1] = &base_B.XYZstageEnc2JntAng_a[1];
          sfcnUPtrs[2] = &base_B.XYZstageEnc2JntAng_a[2];
          ssSetInputPortSignalPtrs(rts, 1, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 1, 1);
          ssSetInputPortWidth(rts, 1, 3);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn12.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 3);
          ssSetOutputPortSignal(rts, 0, ((real_T *) base_B.XYZstageCtrl_h));
        }
      }

      /* path info */
      ssSetModelName(rts, "XYZstageCtrl");
      ssSetPath(rts, "base/Application/XYZstage Control/XYZstageCtrl");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetRWork(rts, (real_T *) &base_DW.XYZstageCtrl_RWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &base_M->NonInlinedSFcns.Sfcn12.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &base_M->NonInlinedSFcns.Sfcn12.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 3);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &base_DW.XYZstageCtrl_RWORK[0]);
      }

      /* registration */
      XYZstageCtrl(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
    }

    /* Level2 S-Function Block: base/<S10>/XYZstageTrq2DA (XYZstageTrq2DA) */
    {
      SimStruct *rts = base_M->childSfunctions[13];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn13.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn13.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn13.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[13]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[13]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[13]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[13]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[13]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &base_M->NonInlinedSFcns.Sfcn13.inputPortInfo[0]);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn13.UPtrs0;
          sfcnUPtrs[0] = base_B.XYZstageCtrl_h;
          sfcnUPtrs[1] = &base_B.XYZstageCtrl_h[1];
          sfcnUPtrs[2] = &base_B.XYZstageCtrl_h[2];
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 3);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn13.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 3);
          ssSetOutputPortSignal(rts, 0, ((real_T *) base_B.XYZstageTrq2DA_p));
        }
      }

      /* path info */
      ssSetModelName(rts, "XYZstageTrq2DA");
      ssSetPath(rts, "base/Application/XYZstage Control/XYZstageTrq2DA");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* registration */
      XYZstageTrq2DA(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: base/<S10>/XYZstageDALimit (XYZstageDALimit) */
    {
      SimStruct *rts = base_M->childSfunctions[14];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn14.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn14.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn14.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[14]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[14]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[14]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[14]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[14]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &base_M->NonInlinedSFcns.Sfcn14.inputPortInfo[0]);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &base_M->NonInlinedSFcns.Sfcn14.UPtrs0;
          sfcnUPtrs[0] = base_B.XYZstageTrq2DA_p;
          sfcnUPtrs[1] = &base_B.XYZstageTrq2DA_p[1];
          sfcnUPtrs[2] = &base_B.XYZstageTrq2DA_p[2];
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 3);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn14.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 3);
          ssSetOutputPortSignal(rts, 0, ((real_T *) base_B.XYZstageDALimit_j));
        }
      }

      /* path info */
      ssSetModelName(rts, "XYZstageDALimit");
      ssSetPath(rts, "base/Application/XYZstage Control/XYZstageDALimit");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetRWork(rts, (real_T *) &base_DW.XYZstageDALimit_RWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &base_M->NonInlinedSFcns.Sfcn14.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &base_M->NonInlinedSFcns.Sfcn14.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* RWORK */
        ssSetDWorkWidth(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &base_DW.XYZstageDALimit_RWORK);
      }

      /* registration */
      XYZstageDALimit(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: base/<Root>/System Setting (SystemSet) */
    {
      SimStruct *rts = base_M->childSfunctions[15];

      /* timing info */
      time_T *sfcnPeriod = base_M->NonInlinedSFcns.Sfcn15.sfcnPeriod;
      time_T *sfcnOffset = base_M->NonInlinedSFcns.Sfcn15.sfcnOffset;
      int_T *sfcnTsMap = base_M->NonInlinedSFcns.Sfcn15.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &base_M->NonInlinedSFcns.blkInfo2[15]);
      }

      ssSetRTWSfcnInfo(rts, base_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &base_M->NonInlinedSFcns.methods2[15]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &base_M->NonInlinedSFcns.methods3[15]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &base_M->NonInlinedSFcns.statesInfo2[15]);
        ssSetPeriodicStatesInfo(rts, &base_M->
          NonInlinedSFcns.periodicStatesInfo[15]);
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &base_M->NonInlinedSFcns.Sfcn15.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 1);
          ssSetOutputPortSignal(rts, 0, ((int32_T *) &base_B.SystemSetting));
        }
      }

      /* path info */
      ssSetModelName(rts, "System Setting");
      ssSetPath(rts, "base/System Setting");
      ssSetRTModel(rts,base_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      ssSetIWork(rts, (int_T *) &base_DW.SystemSetting_IWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &base_M->NonInlinedSFcns.Sfcn15.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &base_M->NonInlinedSFcns.Sfcn15.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* IWORK */
        ssSetDWorkWidth(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &base_DW.SystemSetting_IWORK);
      }

      /* registration */
      SystemSet(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
    }
  }

  {
    /* user code (registration function declaration) */
    /*Initialize global TRC pointers. */
    base_rti_init_trc_pointers();
  }

  /* Initialize Sizes */
  base_M->Sizes.numContStates = (0);   /* Number of continuous states */
  base_M->Sizes.numY = (0);            /* Number of model outputs */
  base_M->Sizes.numU = (0);            /* Number of model inputs */
  base_M->Sizes.sysDirFeedThru = (0);  /* The model is not direct feedthrough */
  base_M->Sizes.numSampTimes = (1);    /* Number of sample times */
  base_M->Sizes.numBlocks = (27);      /* Number of blocks */
  base_M->Sizes.numBlockIO = (25);     /* Number of block outputs */
  base_M->Sizes.numBlockPrms = (49);   /* Sum of parameter "widths" */
  return base_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/

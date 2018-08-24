/*********************** dSPACE target specific file *************************

   Include file base_rti.c:

   Definition of functions and variables for the system I/O and for
   the hardware and software interrupts used.

   RTI1007 7.6 (02-May-2016)
   Fri Aug 03 14:28:41 2018

   Copyright 2018, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

#if !(defined(__RTI_SIMENGINE__) || defined(RTIMP_FRAME))
# error This file may be included only by the RTI(-MP) simulation engine.
#endif

/* Include the model header file. */
#include "base.h"
#include "base_private.h"

/* Defines for block output and parameter structure existence */
#define RTI_rtB_STRUCTURE_EXISTS       1
#define RTI_rtP_STRUCTURE_EXISTS       1
#define RTB_STRUCTURE_NAME             base_B
#define RTP_STRUCTURE_NAME             base_P

/* dSPACE generated includes for header files */
#include <brtenv.h>
#include <rtkernel.h>
#include <rti_assert.h>
#include <rtidefineddatatypes.h>
#include <dsIoEth.h>
#include <rti_sim_engine_exp.h>

/****** Definitions: task functions for timer tasks *********************/

/* Timer Task 1. (Base rate). */
static void rti_TIMERA(rtk_p_task_control_block task)
{
  /* Task entry code BEGIN */
  /* -- None. -- */
  /* Task entry code END */

  /* Task code. */
  baseRateService(task);

  /* Task exit code BEGIN */
  /* -- None. -- */
  /* Task exit code END */
}

/* ===== Definition of interface functions for simulation engine =========== */
#ifdef MULTITASKING
# define dsIsSampleHit(RTM,sti)        rtmGetSampleHitPtr(RTM)[sti]
#else
# define dsIsSampleHit(RTM,sti)        1
#endif

#undef __INLINE
#if defined(_INLINE)
# define __INLINE                      static inline
#else
# define __INLINE                      static
#endif

static void rti_mdl_initialize_host_services(void)
{
  DsDaq_Init(0, 32, 1);
}

static void rti_mdl_initialize_io_boards(void)
{
  /* Registering of RTI products and modules at VCM */
  {
    vcm_module_register(VCM_MID_RTI1007, (void *) 0,
                        VCM_TXT_RTI1007, 7, 6, 0,
                        VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);

    {
      vcm_module_descriptor_type* msg_mod_ptr;
      msg_mod_ptr = vcm_module_register(VCM_MID_MATLAB, (void *) 0,
        VCM_TXT_MATLAB, 9, 0, 0,
        VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);
      vcm_module_register(VCM_MID_SIMULINK, msg_mod_ptr,
                          VCM_TXT_SIMULINK, 8, 7, 0,
                          VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);
      vcm_module_register(VCM_MID_RTW, msg_mod_ptr,
                          VCM_TXT_RTW, 8, 10, 0,
                          VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);
    }
  }

  /* dSPACE I/O Board DSETHERNET #1 */
  {
    /* dSPACE I/O Board DSETHERNET #1 Unit:SETUP Group:SETUP */

    /* set IP address, netmask and gateway of the ethernet interface */
    DsIoEth_setIpAddress("10.1.196.179");
    DsIoEth_setNetMask("255.255.255.0");
    DsIoEth_setGatewayAddress("0.0.0.0");
  }

  /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP */

  /* --- base/ETHERNET_UDP_SETUP_BL1: ==> Socket ID = (0) --- */
  {
    /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP Group:SETUPUDP */
    DsSSockAddrIn localAddr;
    UInt32 iRet, addrLen= sizeof(localAddr);

    /**/
    /* set first application socket to any IP address */
    localAddr.sin_addr.s_addr = (UInt32) DSIOETH_INADDR_ANY;

    /* socket is bound to IP Address */

    /* set application to local port */
    localAddr.sin_port = DsIoEth_htons((UInt16) 50006U);
    localAddr.sin_family = DSIOETH_AF_INET;

    /* creates a non-blocking UDP connection with specified local address *
     * (containing the local IP and port number) and protocol to use.     */
    iRet =
      DsIoEth_create( DSIOETH_CONNECTION_ID_0,
                     (DsSSockAddr*) &localAddr,
                     addrLen,
                     DSIOETH_PROTO_UDP_SERVER,
                     DSIOETH_FLAG_NONE|DSIOETH_FLAG_BIND_TO_INTERFACE_ADDR
                     ) ;
    if (iRet < 0) {
      msg_error_printf(0,0,
                       "Internal Error: Cannot create UDP socket. (dSPACE Support information: DsIoEth_create(SocketID %d) failed!)",
                       0);
      RTLIB_EXIT(1);
    }
  }

  /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP Group:TXUDP */
  /* --- base/Application/UDP Send/ETHERNET_UDP_TX_BL1: ==> Socket ID = (0) --- */
  {
    /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP Group:TXUDP */
    DsSSockAddrIn remoteAddr ;
    UInt8 remoteIp[4];
    UInt32 iRet, addrLen= sizeof(remoteAddr);

    /* use this remote IP for the ARP request */
    remoteIp[0] = 10U;
    remoteIp[1] = 1U;
    remoteIp[2] = 196U;
    remoteIp[3] = 178U;
    remoteAddr.sin_addr.s_addr = *((UInt32*) remoteIp);
    remoteAddr.sin_family = DSIOETH_AF_INET;

    /* ARP request for the remote IP to send to */
    iRet =
      DsIoEth_queryArpEntry( DSIOETH_CONNECTION_ID_0,
      (DsSSockAddr*) &remoteAddr,
      addrLen
      ) ;

    /* error handling */
    if (iRet < 0) {
      msg_error_printf(0,0,
                       "Internal Error: Cannot send ARP request. (dSPACE Support information: DsIoEth_queryArpEntry(UDP SocketID %d) failed!)",
                       0);
      RTLIB_EXIT(1);
    }
  }
}

/* Function rti_mdl_slave_load() is empty */
#define rti_mdl_slave_load()

/* Function rti_mdl_rtk_initialize() is empty */
#define rti_mdl_rtk_initialize()

static void rti_mdl_initialize_io_units(void)
{
  /* dSPACE I/O Board DSETHERNET #1 Unit:RXUDP */
  {
    /* dSPACE I/O Board DSETHERNET #1 Unit:RXUDP Group:RXUDP */

    /* set output Status initially to state 2 indicating empty RX buffer */
    base_B.SFunction1_o2_h = (uint32_T) 4;

    /* set output Received Frames initially to 0 */
    base_B.SFunction1_o3 = (real_T) 0;

    /* fixed frame size mode is adjusted */

    /* configures the frame size used by the application (frame size setting) *
     * if less or more data has been received, nothing will be stored and no  *
     * interrupt will be triggered then.                                      */
    {
      Int32 iRet =
        DsIoEth_setRecvFrameSize( DSIOETH_CONNECTION_ID_0,
        112U,
        DSIOETH_RECV_MODE_EQUAL_SIZE
        );

      /* error handling */
      if (iRet < 0) {
        msg_error_printf(0,0,
                         "Internal Error: Cannot configure UDP frame size filter. (dSPACE Support information: DsIoEth_setRecvFrameSize(SocketID %d) failed!)",
                         0);
        RTLIB_EXIT(1);
      }
    }
  }

  /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP */
  {
    /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP Group:TXUDP */

    /* set output Status initially to state 4 indicating no data has been sent yet during/after init stage */
    base_B.SFunction1_o1 = (uint32_T) 4;

    /* set output Sent Frames initially to 0 */
    base_B.SFunction1_o2 = (real_T) 0;
  }

  /* --- base/ETHERNET_UDP_SETUP_BL1: ==> Socket ID = (0) --- */
  {
    /* dSPACE I/O Board DSETHERNET #1 Unit:SETUPUDP Group:SETUPUDP */
    DsSSockAddrIn remoteAddr ;
    UInt32 iRet, addrLen= sizeof(remoteAddr);
    UInt8 remoteIP[4];
    UInt16 remotePort ;

    /* set remote IP to 0.0.0.0 into the socket structure (to accept any remote IP) */
    remoteIP[0] = 0;
    remoteIP[1] = 0;
    remoteIP[2] = 0;
    remoteIP[3] = 0;

    /* set remote port to 0 into the socket structure (to accept any port) */
    remotePort = 0;

    /**/
    /* set remote IP and port to the socket structure */
    remoteAddr.sin_addr.s_addr = *((UInt32*) remoteIP);
    remoteAddr.sin_port = DsIoEth_htons(remotePort);
    remoteAddr.sin_family = DSIOETH_AF_INET;

    /* configures the socket to the remote station to accept data from.             *
     * If the port number of the socket address. If remote ip address of the socket *
     * address is 0 (INADDR_ANY), then any remote ip address will be accepted.      */
    iRet =
      DsIoEth_setAddrFilter( DSIOETH_CONNECTION_ID_0,
      (DsSSockAddr*) &remoteAddr,
      addrLen
      ) ;

    /* error handling */
    if (iRet < 0) {
      msg_error_printf(0,0,
                       "Internal Error: Cannot configure UDP socket address filter. (dSPACE Support information: DsIoEth_setAddrFilter(SocketID %d) failed!)",
                       0);
      RTLIB_EXIT(1);
    }
  }
}

/* Function rti_mdl_acknowledge_interrupts() is empty */
#define rti_mdl_acknowledge_interrupts()

/* Function rti_mdl_timetables_register() is empty */
#define rti_mdl_timetables_register()

/* Function rti_mdl_timesync_simstate() is empty */
#define rti_mdl_timesync_simstate()

/* Function rti_mdl_timesync_baserate() is empty */
#define rti_mdl_timesync_baserate()

static void rti_mdl_background(void)
{
  /* DsDaq background call */
  DsDaq_Background(0);
}

/* Function rti_mdl_sample_input() is empty */
#define rti_mdl_sample_input()

static void rti_mdl_daq_service()
{
  /* dSPACE Host Service */
  DsDaq_Service(0, 0, 1, (DsDaqSTimestampStruct *)
                rtk_current_task_absolute_time_ptr_get());
}

#undef __INLINE

/****** [EOF] ****************************************************************/

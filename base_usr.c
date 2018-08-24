/******************************************************************************

   Include file base_usr.c:

   Definition of functions for user-defined initialization,
   system I/O, and background process code.

   RTI1005 5.2.5 (18-Mar-2005)
   Wed Feb 15 14:18:53 2006

   Copyright (c) 1997-2003 dSPACE GmbH, GERMANY

 *****************************************************************************/

/* ========================================================================= */
/* =====  Define file version macro. Never change this value.  ============= */
/* ========================================================================= */
#define USER_CODE_FILE_VERSION 5
/* ========================================================================= */

/* Insert #include directives for header files here. */
#include <SysDef.h>
#include <SystemFunc.h>


#if defined(_INLINE)
# define __INLINE static inline
#else
# define __INLINE static
#endif

/////////////////////////////////////////////////
// デモ番号
// Control Desk で値を設定
/////////////////////////////////////////////////
unsigned int     app_num = 0;
#include <Data.h>
DATA	data;


/////////////////////////////////////////////////
// ホストPC通信変数
/////////////////////////////////////////////////
int    host_access_flag = 0;


static void usr_initialize(void)
{
	//int	incount = 0;

/////////////////////////////////////
// ロードした時に1度だけ実行する関数
// 現在はただ1度だけ実行するとうまくいかないのでRUNするたびに初期化している
/////////////////////////////////////
	dspaceInit();
//	dspaceServoOn();
//		ds1005_tic_delay(5.0);
//		msg_error_set(MSG_SM_USER, 1, "Push AVS Emergency Stop Button!");

/////////////////////////////////////
// RUNにするたびに実行する関数
/////////////////////////////////////

}


__INLINE void usr_sample_input(void)
{
}


__INLINE void usr_input(void)
{
}


__INLINE void usr_output(void)
{
}


/////////////////////////////////////////////////
// プログラムをロードするとSTOP状態なので
// この関数が実行されることに注意！！
/////////////////////////////////////////////////
static void usr_terminate(void)
{
	dspaceSetDAZero();
	app_num = 0;
}


static void usr_background(void)
{
}


#undef __INLINE

/****** [EOF] ****************************************************************/

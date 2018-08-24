#define S_FUNCTION_NAME TaskSet
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <stdio.h>
#include <math.h>
#include <common_definition.h>
#include "SysDef.h"
#include "App.h"
#if SYSTEM_HAND
#include "HandFunc.h"
#endif //SYSTEM_HAND
#if SYSTEM_WAM
#include "WAMFunc.h"
#endif //SYSTEM_WAM
#ifndef MATLAB_MEX_FILE
#if SYSTEM_AVS
#include "AVSFunc.h"
#endif //STSTEM_AVS
#endif //MATLAB_MEX_FILE
//#include "Predict3D.h"

#ifndef MATLAB_MEX_FILE
extern unsigned int     app_num;
#else
unsigned int     app_num = 0;
#endif

#if SYSTEM_HAND
extern HAND		hand;
#endif
#if SYSTEM_WAM
extern WAM		wam;
extern WAM		wam2;
#endif
#if SYSTEM_AVS
extern AVSALL	avs;
#endif

#if SYSTEM_XYZ
extern XYZ	xyz;
#endif


static void mdlInitializeSizes(SimStruct *S)
{
    // S-functionへの引数の設定
    ssSetNumSFcnParams(S, 0);
    if(ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))    return;
    // 入力ポートの設定
    if(!ssSetNumInputPorts(S, 0))    return;
    // 出力ポートの設定
    if(!ssSetNumOutputPorts(S, 1))   return;
    ssSetOutputPortWidth(S, 0, 1);
	// ワークベクトルの設定   
	ssSetNumRWork(S, 1);
	// その他
    ssSetNumSampleTimes(S, 1);    
    //ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SYSTEM_INTERVAL);
    ssSetOffsetTime(S, 0, 0.0);
}


#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
	ssSetRWorkValue(S, 0, 0.0);
	// 構造体初期設定
#if SYSTEM_HAND
	handAppSet(&hand);		// ユーザ定義初期化
	handInit(&hand);
#endif
#if SYSTEM_XYZ
    xyzAppSet(&xyz);
#endif
#if SYSTEM_WAM
	wamAppSet(&wam);		// ユーザ定義初期化
	wamInit(&wam);
	wam2AppSet(&wam2);		// ユーザ定義初期化
	wamInit(&wam2);
#endif
#if SYSTEM_AVS
    avsAppSet(&avs);		// ユーザ定義初期化
#ifndef MATLAB_MEX_FILE
	avsInit();
#endif
#endif

////////////////////////////////////////////
// ユーザ定義　ここから
////////////////////////////////////////////
    // 軌道予測初期化
	//predict3D_init();
////////////////////////////////////////////
// ユーザ定義　ここまで
////////////////////////////////////////////
}
#endif


static void mdlOutputs(SimStruct *S,int_T tid)
{
    real_T *app_time = ssGetOutputPortRealSignal(S, 0);
	double	time = ssGetRWorkValue(S, 0);

    *app_time = time;
    if(time < ALL_APP_TIME)		ssSetRWorkValue(S, 0, time+SYSTEM_INTERVAL);		// 時間更新
	else	app_num = 0;       // リセット
}


static void mdlTerminate(SimStruct *S){
////////////////////////////////////////////
// ユーザ定義　ここから
////////////////////////////////////////////
    // 軌道予測終了
	//predict3D_exit();
////////////////////////////////////////////
// ユーザ定義　ここまで
////////////////////////////////////////////
}
    
#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

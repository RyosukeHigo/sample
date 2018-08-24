#define S_FUNCTION_NAME SystemSet
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <stdio.h>
#include <math.h>
#include <SystemFunc.h>
#include <common_definition.h>
//#include "SysDef.h"

///////////////////////////////
// ホストPCとの通信変数
// [model]_usr.c 内で定義
///////////////////////////////
#ifndef MATLAB_MEX_FILE
extern unsigned int     app_num;
#else
unsigned int     app_num = 0;
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
	ssSetOutputPortDataType(S, 0, SS_INT32);
    ssSetOutputPortWidth(S, 0, 1);
	// ワークベクトルの設定   
	ssSetNumIWork(S, 1);
	// その他
    ssSetNumSampleTimes(S, 1);    
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
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
	ssSetIWorkValue(S, 0, 0);
    //app_num = 0;
}
#endif



static void mdlOutputs(SimStruct *S,int_T tid)
{
   
   
    int32_T		*app_trigger= (int32_T*)ssGetOutputPortSignal(S, 0);
    int		flag = ssGetIWorkValue(S, 0);
// ////////////////////////////////////////////////////////////////////
// // Control Desk の Number を指定して RUN を押すと出力が1になる
// ////////////////////////////////////////////////////////////////////
     if(app_num == 0){
 #ifndef MATLAB_MEX_FILE
 	    dspaceSetDAZero();
 #endif
//     //ds1007に変えたせいか， *app_trigger = 0だけだとsimStateがSTOPにならないため一度1を送ったあとに0を送るように変更
        if(flag==0){
            *app_trigger = 1;
            ssSetIWorkValue(S, 0, 1);
        }else{
            *app_trigger = 0;
            //ssSetIWorkValue(S, 0, 0);
        }
     }else{
		// データ保存トリガーを働かせるためわざと1周期遅らせている
        if(flag == 1){
			*app_trigger = 1;
		}else{
			*app_trigger = 0;
			ssSetIWorkValue(S, 0, 1);		// flagを1にする
		}
     }
}


static void mdlTerminate(SimStruct *S){}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

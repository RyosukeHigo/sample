#define S_FUNCTION_NAME SystemSet
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <stdio.h>
#include <math.h>
#include <SystemFunc.h>
#include <common_definition.h>
//#include "SysDef.h"

///////////////////////////////
// �z�X�gPC�Ƃ̒ʐM�ϐ�
// [model]_usr.c ���Œ�`
///////////////////////////////
#ifndef MATLAB_MEX_FILE
extern unsigned int     app_num;
#else
unsigned int     app_num = 0;
#endif

static void mdlInitializeSizes(SimStruct *S)
{
    // S-function�ւ̈����̐ݒ�
    ssSetNumSFcnParams(S, 0);
    if(ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))    return;
    // ���̓|�[�g�̐ݒ�
    if(!ssSetNumInputPorts(S, 0))    return;
    // �o�̓|�[�g�̐ݒ�
    if(!ssSetNumOutputPorts(S, 1))   return;
	ssSetOutputPortDataType(S, 0, SS_INT32);
    ssSetOutputPortWidth(S, 0, 1);
	// ���[�N�x�N�g���̐ݒ�   
	ssSetNumIWork(S, 1);
	// ���̑�
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
// // Control Desk �� Number ���w�肵�� RUN �������Əo�͂�1�ɂȂ�
// ////////////////////////////////////////////////////////////////////
     if(app_num == 0){
 #ifndef MATLAB_MEX_FILE
 	    dspaceSetDAZero();
 #endif
//     //ds1007�ɕς����������C *app_trigger = 0��������simState��STOP�ɂȂ�Ȃ����߈�x1�𑗂������Ƃ�0�𑗂�悤�ɕύX
        if(flag==0){
            *app_trigger = 1;
            ssSetIWorkValue(S, 0, 1);
        }else{
            *app_trigger = 0;
            //ssSetIWorkValue(S, 0, 0);
        }
     }else{
		// �f�[�^�ۑ��g���K�[�𓭂����邽�߂킴��1�����x�点�Ă���
        if(flag == 1){
			*app_trigger = 1;
		}else{
			*app_trigger = 0;
			ssSetIWorkValue(S, 0, 1);		// flag��1�ɂ���
		}
     }
}


static void mdlTerminate(SimStruct *S){}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

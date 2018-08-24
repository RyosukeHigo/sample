#include <stdio.h>
#include <math.h>
#include <common_definition.h>
#include <WAM.h>
#include <WAMFunc.h>
#include "App.h"
#include "SysDef.h"
#include "Data.h"
#include "Predict3D.h"

///////////////////////////////
// ホストPCとの通信変数
// [model]_usr.c 内で定義
///////////////////////////////
#ifndef MATLAB_MEX_FILE

#else

#endif

//　batting_ball_count の値と状態のカウントの対応
enum {	
	BC_STRIKE = 1,		//ストライク
	BC_BALL,			//ボール
	BC_BALL_DZONE,		//ボール（危険領域のため）
	BC_BALL_INSWING,	//ボール（スウィング中にボールになった）
};

// 構造体宣言
WAM		wam;


// グローバル座標におけるアーム位置
static HomoMat wam_base_homo ={{1.0, 0.0, 0.0},
		                        {0.0, 1.0, 0.0},
		                        {0.0, 0.0, 1.0},
								{0.0, 0.675, 0.0}};		// 定盤から1軸中心位置までの高さ84.675cm, 定盤厚さは5cm, ただし床面から定盤表面までは5.5cm(床の凹凸により多少浮いている)
// 床からのz=0までの高さ90.175cm

// 制御ゲイン
static WAMCtrlCoef wam_ctrl_coef = {{8.0, 8.0, 8.0, 8.0, 1000.0},				// Kp
									{0.0, 0.0, 0.0, 0.0, 0.0},					// Ti
									{120.0, 120.0, 80.0, 80.0, 3000.0},			//Td
									{0.0, 0.0, 0.0, 0.0, 0.0},					// Cf
									100};										// Kg
// 初期姿勢
const static WAMJnt   prepare_jnt_ang = {-PI/2.0+0.1, PI/5.0, -3.0*PI/4.0, 7.0*PI/12.0}; 

int positionFlag; //ボールが通りすぎたかどうか　20090618

//////////////////////////////////////////////////////
int least_square(double *x_coef, double *y_coef, double *obj_pos, int count, int flag)
{
	static double	sum_x = 0;
	static double	sum_y = 0;
	static double	sum_t = 0, sum_tx = 0, sum_ty = 0, sum_t2 = 0;
	static int		num = 0;
	double	denominator, time;

	// リセット
	if(flag == -1){
		sum_x = 0;		sum_y = 0;
		sum_t = 0; sum_tx = 0; sum_ty = 0; sum_t2 = 0;
		num = 0;
		return	0;
	}

	time = count * 0.001;
	num++;

	sum_x += obj_pos[0];
	sum_y += obj_pos[1];
	sum_t += time;
	sum_tx += time*obj_pos[0];
	sum_ty += time*obj_pos[1];
	sum_t2 += time*time;

	// 初回
	if(flag == 0)	return 0;

	denominator = num*sum_t2-sum_t*sum_t;
	x_coef[0] = (num*sum_tx-sum_t*sum_x) / denominator;
	x_coef[1] = (sum_t2*sum_x-sum_tx*sum_t) / denominator;
	y_coef[0] = (num*sum_ty-sum_t*sum_y) / denominator;
	y_coef[1] = (sum_t2*sum_y-sum_ty*sum_t) / denominator;

//printf("%d\n", count);
	return 0;
}

//////////////////////////////////////////////////////
// バッティング
//////////////////////////////////////////////////////
int wamTrajApp(WAMJnt ref_jnt_ang, DATA *data, double time, int appNum)
{
	int jnt;
	for(jnt=0; jnt<WAM_JNT; jnt++){
		ref_jnt_ang[jnt] = prepare_jnt_ang[jnt];
	}
	return 0;
}


//////////////////////////////////////
// アーム初期設定
//////////////////////////////////////
int wamAppSet(WAM *wam)
{
	int jnt;
//////////////////////////////////////
	wam->cons.da_limit = 1.0;		// 0.0 ～ 1.0（この値にDA_LIMIT_WAMをかけたDA指令を出力）
//////////////////////////////////////
	wam->base_homo = &wam_base_homo;
	wam->ctrl_coef = &wam_ctrl_coef;
	for(jnt=0; jnt<WAM_JNT; jnt++){
		wam->prepare_jnt_ang[jnt] = prepare_jnt_ang[jnt];
		wam->past_var.jnt_ang[jnt] = prepare_jnt_ang[jnt];
		wam->past2_var.jnt_ang[jnt] = prepare_jnt_ang[jnt];
		wam->past_ref_var.jnt_ang[jnt] = prepare_jnt_ang[jnt];
		wam->past2_ref_var.jnt_ang[jnt] = prepare_jnt_ang[jnt];
	}
	return 0;
}

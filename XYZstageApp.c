/*
#ifndef MATLAB_MEX_FILE
	extern double Kp_x;
	extern double Td_x;
	extern double Kp_y;
	extern double Td_y;
#else
	extern double Kp_x = 0.0;
	extern double Td_x = 0.0;
	extern double Kp_y = 0.0;
	extern double Td_y = 0.0;
#endif
*/
#include <stdio.h>
#include <math.h>
#include <XYZstage.h>
#include "App.h"

// 構造体宣言
XYZ xyz;

// グローバル座標におけるハンド位置
static HomoMat xyz_base_homo = {{1.0, 0.0, 0.0},
							   {0.0, 1.0, 0.0},
							   {0.0, 0.0, 1.0},
							   {0.0, 0.0, 0.0}};
// 制御ゲイン
XYZCtrlCoef xyz_ctrl_coef = { { 60.0, 60.0, 7.0},   // Kp
							{ 0.0, 0.0, 0.0 },	 // Ti
							{ 950.0, 950.0, 126.0}, // Td
							{ 0.0, 0.0, 0.0 },	 // Cf
							1.7 };		   // Kg

//時間の定義
#define T_START 1.0  //停止時間
#define T_END 1.5	//動作時間
#define T_END1 2.0   //動作時間
#define T_END2 2.5   //停止時間
#define T_END3 3.0   //停止時間
#define T_END4 3.5   //停止時間
#define T_END5 4.0   //停止時間
#define T_END6 4.5   //停止時間
#define T_END7 5.0   //停止時間
#define T_END8 5.5   //停止時間
#define T_END9 6.0   //停止時間
#define T_END10 15.0 //停止時間

//周波数の定義
#define OMEGA 1.0

// 初期姿勢
static XYZJnt prepare_jnt_pos = { 0.3615, 0.0, 0.025};
XYZJnt prev_jnt_pos = {0.3615, 0.0, 0.025};

//////////////////////////////////////////////////////
int xyzTrajApp(XYZJnt ref_jnt_pos, XYZJnt data, double *obj, double time, double *sensor, double *camera)
{
	int jnt, crd;
	double camX;
	double camY;
	double width = 192.0;
	double hight = 600.0;

	double pixel_h_m = 0.27 / 59;
	double pixel_w_m = 0.525 / 192;//画像上の1pxが実際の何mか

	//取得画像上でのゴルフボールの重心位置の初期位置の座標
	double center_x = width / 2;
	double center_y = 296 - 0.025 / pixel_h_m;

	double target_x = camera[1] - center_x;
	double target_y = camera[2] - center_y;
	

	// 初期位置
	for (jnt = 0; jnt < XYZ_JNT; jnt++)
		ref_jnt_pos[jnt] = prepare_jnt_pos[jnt];

	// 動作軌道
	/*
	if(time <= T_START){
		ref_jnt_pos[XY_M1] = 0.1;
		ref_jnt_pos[XY_M3] = 0.01;
	}else if(time <= T_END){
		ref_jnt_pos[XY_M1] = 0.1+0.2*(time-T_START)/(T_END-T_START);
		ref_jnt_pos[XY_M3] = 0.01+0.02*(time - T_START) / (T_END - T_START);
	}else if(time <= T_END1){
		ref_jnt_pos[XY_M1] = 0.3;
		ref_jnt_pos[XY_M3] = 0.03;
	}else if(time <= T_END2){
		ref_jnt_pos[XY_M1] = 0.3-0.2*(time-T_END1)/(T_END2-T_END1);
		ref_jnt_pos[XY_M3] = 0.03-0.02*(time - T_END1) / (T_END2 - T_END1);
	}else if(time <= T_END3){
		ref_jnt_pos[XY_M1] = 0.1;
		ref_jnt_pos[XY_M3] = 0.01;
	}else if(time <= T_END4){
		ref_jnt_pos[XY_M1] = 0.1+0.4*(time-T_END3)/(T_END4-T_END3);
		ref_jnt_pos[XY_M3] = 0.01;
	}else if(time <= T_END5){
		ref_jnt_pos[XY_M1] = 0.5;
		ref_jnt_pos[XY_M3] = 0.01;
	}else if(time <= T_END6){
		ref_jnt_pos[XY_M1] = 0.5-0.4*(time-T_END5)/(T_END6-T_END5);
		ref_jnt_pos[XY_M3] = 0.01;
	}else if(time <= T_END7){
		ref_jnt_pos[XY_M1] = 0.1;
		ref_jnt_pos[XY_M3] = 0.01;
	}else if(time <= T_END8){
		ref_jnt_pos[XY_M1] = 0.1+0.5*(time-T_END7)/(T_END8-T_END7);
		ref_jnt_pos[XY_M3] = 0.01;
	}else if(time <= T_END9){
		ref_jnt_pos[XY_M1] = 0.6;
		ref_jnt_pos[XY_M3] = 0.01;
	}else{
		ref_jnt_pos[XY_M1] = 0.6;
		ref_jnt_pos[XY_M3] = 0.01;
	}
	*/
	//ref_jnt_pos[XY_M1] = 0.05;
	//ref_jnt_pos[XY_M3] = 0.02;
#if 1
	// カメラ情報を用いたテスト
	camX = 0;
	camY = 0;
	//ヨコの目標位置計算
	//初期位置から左右10cmまでなら追従して、それ以外なら止まる
	if (target_x * pixel_w_m < -0.10 || target_x * pixel_w_m > 0.10)
	{
		ref_jnt_pos[XYZ_M1] = prev_jnt_pos[XYZ_M1];
	}
	else
	{
		ref_jnt_pos[XYZ_M1] = prepare_jnt_pos[XYZ_M1] + target_x * pixel_w_m;
	}
	//タテの目標位置計算
	//初期位置から上下2cmまでなら追従して、それ以外なら止まる
	if (-target_y * pixel_h_m < -0.020 || -target_y * pixel_h_m > 0.020)
	{
		ref_jnt_pos[XYZ_M3] = prev_jnt_pos[XYZ_M3];
	}
	else
	{
		ref_jnt_pos[XYZ_M3] = prepare_jnt_pos[XYZ_M3] - target_y * pixel_h_m;
	}

	prev_jnt_pos[XYZ_M1] = ref_jnt_pos[XYZ_M1];
	prev_jnt_pos[XYZ_M3] = ref_jnt_pos[XYZ_M3];
	/*
	if (time <= T_START)
	{
		ref_jnt_pos[XYZ_M1] = prepare_jnt_pos[XYZ_M1];
		ref_jnt_pos[XYZ_M3] = prepare_jnt_pos[XYZ_M3];
	}
	else if (time <= T_END10)
	{
		ref_jnt_pos[XYZ_M1] = prepare_jnt_pos[XYZ_M1] + camX;
		ref_jnt_pos[XYZ_M3] = prepare_jnt_pos[XYZ_M3];
	}
	else
	{
		ref_jnt_pos[XYZ_M1] = prepare_jnt_pos[XYZ_M1];
		ref_jnt_pos[XYZ_M3] = prepare_jnt_pos[XYZ_M3];
	}
	
	//camX = 0.055;
	if (time <= T_START)
	{
		ref_jnt_pos[XYZ_M1] = prepare_jnt_pos[XYZ_M1];
		ref_jnt_pos[XYZ_M3] = prepare_jnt_pos[XYZ_M3];
	}
	else if (time <= T_END10)
	{
		ref_jnt_pos[XYZ_M1] = prepare_jnt_pos[XYZ_M1] + camX;
		ref_jnt_pos[XYZ_M3] = prepare_jnt_pos[XYZ_M3];
	}
	else
	{
		ref_jnt_pos[XYZ_M1] = prepare_jnt_pos[XYZ_M1];
		ref_jnt_pos[XYZ_M3] = prepare_jnt_pos[XYZ_M3];
	}
	*/
#endif
	
	return 0;
}

//////////////////////////////////////
// ハンド�?�期設�?
//////////////////////////////////////
int xyzAppSet(XYZ *xyz)
{
	int jnt;
	xyz->base_homo = &xyz_base_homo;
	xyz->ctrl_coef = &xyz_ctrl_coef;
	xyz->prepare_jnt_pos = prepare_jnt_pos;
	return 0;
}

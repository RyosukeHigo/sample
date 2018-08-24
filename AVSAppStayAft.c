#include <stdio.h>
#include <math.h>
#include <AVS.h>
#include "App.h"

// 構造体宣言
AVSALL	avs;

// グローバル座標におけるAVS位置
/*
static HomoMat avs_base_homo[AVS_NUM] ={{{0.0, -1.0, 0.0},
										{1.0, 0.0, 0.0},
		                                {0.0, 0.0, 1.0},
										{1.45, 0.375, 0.14}},
										{{0.0, -1.0, 0.0},
										{1.0, 0.0, 0.0},
										{0.0, 0.0, 1.0},
										{1.45, -0.375, 0.14}}};
*/
static HomoMat avs_base_homo[AVS_NUM] ={{{0.0, -1.0, 0.0},
										{1.0, 0.0, 0.0},
		                                {0.0, 0.0, 1.0},
//										{1.438, 0.4, 0.148}},	// AVS取り付け位置・・アーム側の定盤穴から4つ目
//										{1.538, 0.4, 0.148}},	// AVS取り付け位置・・アーム側の定盤穴から8つ目
										//{1.588, 0.4, 0.148}},	// AVS取り付け位置・・アーム側の定盤穴から10つ目
										//{1.588, 0.4, 0.348}},	// 人投げ用に高くした 2010.06.02
										{1.665, 0.4, 0.148}},	// WAMの定盤がずれた　2010.11.15
										{{0.0, -1.0, 0.0},
										{1.0, 0.0, 0.0},
										{0.0, 0.0, 1.0},
//										{1.438, -0.4, 0.148}}};		// 定盤のx軸方向の端はx=0.75
//										{1.538, -0.4, 0.148}}};		// 定盤のx軸方向の端はx=0.75
										//{1.588, -0.4, 0.148}}};		// 定盤のx軸方向の端はx=0.75
										//{1.588, -0.4, 0.348}}};		// 人投げ用に高くした 2010.06.02
										{1.665, -0.4, 0.148}}};	// WAMの定盤がずれた　2010.11.15
// 床から定盤表面まで高さ54cm, やぐら30cm, チルト軸までの高さ21cm
// 床からチルト軸までの高さ105cm - 床からアーム原点までの高さ90.175cm = 14.825cm
// 定盤端まで75cm + 定盤端からAVS防振台脚まで52cm 防振台脚からAVS定盤端 -0.9cm AVS定盤端から1つ目の穴まで+3.7cm　やぐら取り付け位置+7.5cm　やぐら取り付け穴からAVS軸中心+6.5 = 143.8cm


// 制御ゲイン
#if 0

static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 600.0, 600.0, 600.0},    // Kp
									   {0.0, 0.0, 0.0, 0.0},    // Ti
									   {14000.0, 20000.0, 20000.0, 20000.0},      // Td
									   {7.0, 7.0, 10.0, 10.0},     // IcKp：ビジュアルサーボゲイン
									   50.0};                 // Kg


#else	// 20090325
//static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 600.0, 600.0, 600.0},    // Kp
//									   {0.0, 0.0, 0.0, 0.0},    // Ti
//									   {12000.0, 20000.0, 20000.0, 20000.0},      // Td
//									   {7.0, 7.0, 10.0, 10.0},     // IcKp：ビジュアルサーボゲイン
//									   50.0};                 // Kg
// 20090721
//static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 600.0, 400.0, 400.0},    // Kp
//									   {0.0, 0.0, 0.0, 0.0},    // Ti
//									   {12000.0, 20000.0, 14000.0, 14000.0},      // Td
//									   {7.0, 7.0, 10.0, 10.0},     // IcKp：ビジュアルサーボゲイン
//									   50.0};                 // Kg
// 20091113
//static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 500.0, 400.0, 400.0},    // Kp
//									   {0.0, 0.0, 0.0, 0.0},    // Ti
//									   {12000.0, 15000.0, 14000.0, 14000.0},      // Td
//									   {6.5, 6.5, 6.5, 6.5},     // IcKp：ビジュアルサーボゲイン
//									   50.0};                 // Kg

// 20091114
//static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 500.0, 400.0, 400.0},    // Kp
//									   {0.0, 0.0, 0.0, 0.0},    // Ti
//									   {12000.0, 15000.0, 14000.0, 14000.0},      // Td
//									   {5.5, 6.5, 6.5, 6.5},     // IcKp：ビジュアルサーボゲイン
//									   40.0};                 // Kg
//20091202
static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 500.0, 400.0, 400.0},    // Kp
									   {0.0, 0.0, 0.0, 0.0},    // Ti
									   {12000.0, 15000.0, 14000.0, 14000.0},      // Td
									   {6.5, 6.5, 6.5, 6.5},     // IcKp：ビジュアルサーボゲイン
									   50.0};                 // Kg

//static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 500.0, 500.0, 400.0},    // Kp
//									   {0.0, 0.0, 0.0, 0.0},    // Ti
//									   {12000.0, 15000.0, 14500.0, 14000.0},      // Td
//									   {7.0, 7.0, 8.0, 8.0},     // IcKp：ビジュアルサーボゲイン
//									   50.0};                 // Kg

#endif
// 初期姿勢
//static AVSJntAll	avs_prepare_jnt_ang = {PI/4, -PI/4, PI/4, PI/4};		// スローイングアーム方向(人が投げる時)
//static AVSJntAll	avs_prepare_jnt_ang = {PI/5, -PI/3, PI/5, PI/3};		// スローイングアーム方向(WAM2が投げる時)
//static AVSJntAll	avs_prepare_jnt_ang = {PI/5, -PI/3+0.07, PI/5, PI/3-0.07};		// スローイングアーム方向(WAM2が投げる時)
//  static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/12, -PI/3, PI/5-PI/12, PI/3};		// スローイングアーム方向(WAM2が投げる時) 091027
//  static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/24, -PI/3, PI/5-PI/24, PI/3};		// スローイングアーム方向(WAM2が投げる時)
//  static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/12, -PI/3, PI/5-PI/12, PI/3};		// スローイングアーム方向(WAM2が投げる時)
//static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/24, PI/3, PI/5-PI/24, -PI/3};		// バッティングアーム方向(人が投げる（デモをする）時)
//  static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/12+0.1, -PI/3, PI/5-PI/12+0.1, PI/3};		// スローイングアーム方向(WAM2が投げる時) 100728 調整
  //static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/12+0.03, -PI/3, PI/5-PI/12+0.03, PI/3};		// スローイングアーム方向(WAM2が投げる時) 110418 調整
//  static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/12+0.03, -PI/3-0.1521, PI/5-PI/12+0.03-0.0259, PI/3+0.1652+0.0264};		// 111109 調整
  static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/12, -PI/3-0.1721, PI/5-PI/12+0.03, PI/3+0.2116};		// 120326 調整



#ifndef MATLAB_MEX_FILE
extern int positionFlag; //ボールが通り過ぎたかどうか 
#else
  int positionFlag;
#endif

//static AVSJntAll avs_last_jnt_ang = {PI/4, -PI/3, PI/4, PI/3}; //avsの最終姿勢
//static AVSJntAll avs_last_jnt_ang = {PI/3, -PI/3, PI/3, PI/3}; //avsの最終姿勢
static AVSJntAll avs_last_jnt_ang = {PI/3, PI/3, PI/3, -PI/3}; //avsの最終姿勢
//////////////////////////////////////////////////////
int avsTrajApp(AVSJntAll ref_jnt_ang, double time)
{
	int jnt;
/////////////////////////////
// 対象を見失ったときの軌道
//
	//見えてすぐ見失った時は初期位置に戻るが，
	//ある程度見えた後の場合は1フレーム前の位置に留まる．
	//ref_jnt_angはこの関数に入る前に1フレーム前の値で初期化してるため
	//何も代入しなければ，1個前の関節角になる．
/////////////////////////////
	if(positionFlag == 0){
		for(jnt=0;jnt<AVS_JNT_ALL;jnt++){
			ref_jnt_ang[jnt] = avs_prepare_jnt_ang[jnt];
		}
	}
		return 0;
  }

////////////////////////
// ビジュアルサーボ
////////////////////////
#if 1
int avsVisualTrajApp(AVSImgCenAll ref_img_center, const AVSImgCenAll img_center, double time)
{
	// 対象が画像中心にくるように制御
	ref_img_center[AVS1_PAN] = 0.0;
	ref_img_center[AVS1_TILT] = 0.0;
	ref_img_center[AVS2_PAN] = 0.0;
	ref_img_center[AVS2_TILT] = 0.0;
	return 0;
}

#else
////////////////////////
// バッティング用ビジュアルサーボ
////////////////////////
int avsVisualTrajApp(AVSImgCenAll ref_img_center, const AVSImgCenAll img_center, double time)
{
#define NON_MOVABLE     0
#define MOVABLE         1
#define APPROACH_TIME 100		// [ms]

    static int m_flag[AVS_NUM], incount[AVS_NUM];
    static double init_y[AVS_NUM];

    // 初期化
    if(time < 0.0025){ incount[AVS1] = 0; m_flag[AVS1] = NON_MOVABLE;}
	if(img_center[IMG1_VISIBLE] > 0){
		/////// x座標 ////////
		if(m_flag[AVS1] == NON_MOVABLE){
			ref_img_center[AVS1_PAN] = avs_prepare_jnt_ang[0];
			if(img_center[AVS1_PAN] > 0) m_flag[AVS1] = MOVABLE;
		}else{
			ref_img_center[AVS1_PAN] = 0.0;
		}
		/////// y座標 ////////
		if(incount[AVS1] == 0){
			init_y[AVS1] = img_center[AVS1_TILT];
			ref_img_center[AVS1_TILT] = init_y[AVS1];
		}else if(incount[AVS1] < APPROACH_TIME){
			ref_img_center[AVS1_TILT] = (APPROACH_TIME-incount[AVS1])*init_y[AVS1]/APPROACH_TIME;
		}else{
			ref_img_center[AVS1_TILT] = 0.0;
		}
		incount[AVS1]++;
	}

    // 初期化
    if(time < 0.0025){ incount[AVS2] = 0; m_flag[AVS2] = NON_MOVABLE;}
	if(img_center[IMG2_VISIBLE] > 0){
		/////// x座標 ////////
		if(m_flag[AVS2] == NON_MOVABLE){
			ref_img_center[AVS2_PAN] = avs_prepare_jnt_ang[0];
			if(img_center[AVS2_PAN] < 0) m_flag[AVS2] = MOVABLE;
		}else{
			ref_img_center[AVS2_PAN] = 0.0;
		}
		/////// y座標 ////////
		if(incount[AVS2] == 0){
			init_y[AVS2] = img_center[AVS2_TILT];
			ref_img_center[AVS2_TILT] = init_y[AVS2];
		}else if(incount[AVS2] < APPROACH_TIME){
			ref_img_center[AVS2_TILT] = (APPROACH_TIME-incount[AVS2])*init_y[AVS2]/APPROACH_TIME;
		}else{
			ref_img_center[AVS2_TILT] = 0.0;
		}
		incount[AVS2]++;
	}

	return 0;
}
#endif

//////////////////////////////////////
// アクティブビジョン初期設定
//////////////////////////////////////
int avsAppSet(AVSALL *avs)
{
	int jnt, num;
//////////////////////////////////////
	avs->cons.da_limit = 0.8;		// 0.0 ～ 1.0（この値にDA_LIMIT_AVSをかけたDA指令を出力）
//////////////////////////////////////
	avs->base_homo = avs_base_homo;
	avs->ctrl_coef = &avs_ctrl_coef;
	avs->prepare_jnt_ang = avs_prepare_jnt_ang;
	return 0;
}

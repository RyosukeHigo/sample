#include <stdio.h>
#include <math.h>
#include <common_definition.h>
#include <Hand.h>
#include "App.h"
///////////////////////////////
// ホストPCとの通信変数
// [model]_usr.c 内で定義
///////////////////////////////
#ifndef MATLAB_MEX_FILE
#else
#endif

// 構造体宣言
HAND hand;

// グローバル座標におけるハンド位置
static HomoMat hand_base_homo = {{1.0, 0.0, 0.0},
								 {0.0, 1.0, 0.0},
								 {0.0, 0.0, 1.0},
								 {0.0, 0.0, 0.0}};
// 制御ゲイン
//2011109
//HANDCtrlCoef hand_ctrl_coef = {{12.0, 13.0, 6.0, 13.0, 6.0, 7.0, 11.0, 11.0, 9.5, 10.0, 7.0},	// Kp   9&10=7.0 6=11.0 2=7.0 ,11=12.0
//								{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},				// Ti
//								{27.0, 20.0, 23.0, 23.0, 23.0, 23.0, 35.0, 40.0, 18.0, 20.0, 13.0},	// Td   9&10=17.0 6=27.0, 11=70.0
//								{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},				// Cf
//								2.0};		//4.0		//Kg
//20180613
/*
HANDCtrlCoef hand_ctrl_coef = {{12.0, 10.0, 8.0, 10.0, 12.0, 10.0, 11.0, 11.0, 9.5, 10.0, 7.0},	// Kp   9&10=7.0 6=11.0 2=7.0 ,11=12.0
								{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},				// Ti
								{27.0, 20.0, 18.0, 20.0, 27.0, 20.0, 35.0, 35.0, 18.0, 20.0, 13.0},	// Td   9&10=17.0 6=27.0, 11=70.0
								{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},				// Cf
								2.0};		//4.0		//Kg
*/
//hand2013
HANDCtrlCoef hand_ctrl_coef = {{15.0, 20.0, 15.0, 20.0, 15.0, 20.0, 22.0, 22.0, 19.0, 20.0, 14.0}, // Kp
							   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},			   // Ti
							   {60.0, 75.0, 50.0, 75.0, 60.0, 75.0, 70.0, 70.0, 36.0, 40.0, 26.0}, // Td
							   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},			   // Cf
							   1.0};
//////////////////////////////////////////////////////////////////////////////
// アームに搭載してハンドの手首屈曲0を初期姿勢にすると重力で落ちてしまう．
// そこでハンドの指が水平になるように補正したときの手首初期オフセットを設定．
//////////////////////////////////////////////////////////////////////////////
#define WRIST_HORIZEN_OFFSET (-0.7496)

// 初期姿勢
static HANDJnt init_jnt_ang = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
#define LEFT_FINGER 0
#define MIDDLE_FINGER 1
#define RIGHT_FINGER 2
#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0

// 回転記号 記号を数値へ変換
#define U_TURN_P 1
#define X_ROTATION 2
#define Y_ROTATION 3
#define Y_ROTATION_P 4

//回転記号の実行時間
#define TIME_U_P 0.40
#define TIME_X 0.45
#define TIME_Y 0.2
//const double PI = 3.14159265359;
//ハンドのリンクの長さ2018-01-19
const double Lr = 0.030;
const double L1 = 0.062;		 //指付け根リンク長 //古い方のハンドは0.062[m] hand2013のやつは0.0639[m]
								 //const double L2 = 0.0365;//指先リンク長　半球の中心を指先とする場合は0.0365m　指先の場合0.045m
								 //const double L2 = 0.049; //指サックつけたとき
const double r = 0.045 - 0.0365; //指先半球の半径
const double Lpr = 0.03;
const double Lpm = 0.0;
const double Lpl = -0.03;
//const double Lw0 = 0.105;
const double Lw1 = 0.045; //hand2013の設計図の0.045という値は正しくない　実際に測ると0.0515となった 2018-07-25に測ったら0.045が正しいかもしれない
						  //重心位置をグローバルで保存cog_writeで書き込み制限
double cog_x = 0.0;
double cog_y = 0.0;
int cog_write = 1;

//前のハンド関節の値を保存
double prev_jnt_ang[10];

double prev_mfinger_x;
double prev_mfinger_y;
double prev_lfinger_x;
double prev_lfinger_y;
double prev_rfinger_x;
double prev_rfinger_y;
//運動学で関節角から右指、左指の指先座標を求める
/*
*  同次変換行列
*     T={{nx, ox, ax, px},
*        {ny, oy, ay, py},
*        {nz, oz, az, pz},
*        { 0,  0,  0,  1}};
*/
/*void forKine(HANDJnt ref_jnt_ang, HANDJnt prepare_jnt_ang1, char finger_type)
{
double qw1 = ref_jnt_ang[HAND02_M10];//手首屈曲関節
double qw2 = ref_jnt_ang[HAND02_M9];//手首旋回関節
double q1 = 0;
double q2 = 0;
double q3 = 0;
double Lp = 0;
//左指
if(finger_type == 'r')
{
q1 = ref_jnt_ang[HAND02_M7];
q2 = ref_jnt_ang[HAND02_M1];
q3 = ref_jnt_ang[HAND02_M2];
Lp = Lpr;
}
//中指
else if(finger_type == 'm')
{
q1 = 0;
Lp = Lpm;
}
//右指
else if(finger_type == 'l')
{
q1 = ref_jnt_ang[HAND02_M8];
q2 = ref_jnt_ang[HAND02_M3];
q3 = ref_jnt_ang[HAND02_M4];
Lp = Lpl;
}
HomoMat T;
T.nvec[0] = sin(qw1); T.ovec[0] = cos(qw1)*sin(qw2);  T.avec[0] = cos(qw1)*cos(qw2);  T.pvec[0] = 0;
T.nvec[1] = 0; 		  T.ovec[1] = -cos(qw2);          T.avec[1] = sin(qw2);           T.pvec[1] = 0;
T.nvec[2] = cos(qw1); T.ovec[2] = -sin(qw1)*sin(qw2); T.avec[2] = -sin(qw1)*cos(qw2); T.pvec[2] = 0;
HomoMat A0;
A0.nvec[0] = 1; A0.ovec[0] = 0; A0.avec[0] = 0; A0.pvec[0] = Lw0;
A0.nvec[1] = 0; A0.ovec[1] = 1; A0.avec[1] = 0; A0.pvec[1] = Lp;
A0.nvec[2] = 0; A0.ovec[2] = 0; A0.avec[2] = 1; A0.pvec[2] = Lw1;
HomoMat A1;
A1.nvec[0] = cos(q1); A1.ovec[0] = 0; A1.avec[0] = sin(q1);  A1.pvec[0] = Lr*cos(q1);
A1.nvec[1] = sin(q1); A1.ovec[1] = 0; A1.avec[1] = -cos(q1); A1.pvec[1] = Lr*sin(q1);
A1.nvec[2] = 0;       A1.ovec[2] = 1; A1.avec[2] = 0;        A1.pvec[2] = 0;
HomoMat A2;
A2.nvec[0] = -sin(q2);  A2.ovec[0] = -cos(q2); A2.avec[0] = 0; A2.pvec[0] = -L1*sin(q2);
A2.nvec[1] = cos(q2);   A2.ovec[1] = -sin(q2); A2.avec[1] = 0; A2.pvec[1] = L1*cos(q2);
A2.nvec[2] = 0;         A2.ovec[2] = 0;        A2.avec[2] = 1; A2.pvec[2] = 0;
HomoMat A3;
A3.nvec[0] = cos(q3); A3.ovec[0] = -sin(q3); A3.avec[0] = 0; A3.pvec[0] = L2*cos(q3);
A3.nvec[1] = sin(q3); A3.ovec[1] = cos(q3);  A3.avec[1] = 0; A3.pvec[1] = L2*sin(q3);
A3.nvec[2] = 0;       A3.ovec[2] = 0;       A3.avec[2] = 1; A3.pvec[2] = 0;

HomoMat *hand0,*hand1,*hand2,*hand3,*hand4;
homoMatProduct(hand0, &T,  &A0);
homoMatProduct(hand1, hand0, &A1);
homoMatProduct(hand2, hand1, &A2);
homoMatProduct(hand3, hand2, &A3);
ref_jnt_ang[HAND02_M8] = hand3->pvec[2];
ref_jnt_ang[HAND02_M3] = hand3->pvec[0];
ref_jnt_ang[HAND02_M4] = hand3->pvec[1];
}
*/

//逆運動学で右指、左指の角度を求める
//並木先生のキネマティクスのドキュメントとはハンド屈曲関節の回転方向の定義が逆だったのでqw1の符号を反転させた
//解がないときは前の値を代入するようにした
void invKine(HANDJnt ref_jnt_ang, HANDJnt prepare_jnt_ang1, double time, double stime, double angle, double xm, double ym, double zm, int finger_type)
{
	//
	//手首屈曲関節
	double L2 = 0.0365;
	double Lw0 = 0.07; //hand2013はLw0 = 0.105
	double Lpr = 0.03;
	double Lpl = -0.03;
	double Lp = 0.03;
	double qw1 = 0.0;
	double ta = 0.0;
	double tb = 0.0;
	double C3 = 0.0;
	double S3 = 0.0;
	double q3 = 0.0;
	double A = 0.0;
	double B = 0.0;
	double C = 0.0;
	double q2 = 0.0;
	if (finger_type == MIDDLE_FINGER)
	{
		L2 = 0.0525 - 0.0085; //指先の半球の半径0.0085mを引いた長さ
	}
	Lw0 = 0.07; //0.105;
	if (finger_type == MIDDLE_FINGER)
	{
		Lw0 += 0.03;
	}
	Lpr = 0.03;
	Lpl = -0.03;
	Lp = 0.03;
	if (finger_type == RIGHT_FINGER)
	{
		Lp = Lpr;
	}
	else if (finger_type == LEFT_FINGER)
	{
		Lp = Lpl;
	}
	else if (finger_type == MIDDLE_FINGER)
	{
		Lp = Lpl;
	}
	qw1 = (atan2(xm, zm) - atan2(sqrt(xm * xm + zm * zm - Lw0 * Lw0), Lw0)); //xm * xm + zm * zm - Lw0 * Lw0>=0
	if (xm * xm + zm * zm - Lw0 * Lw0 < 0.0)
	{
		return;
	}
	//指先関節
	ta = xm * cos(qw1) - zm * sin(qw1) - Lw1;
	tb = ym - Lr + Lp;
	if (finger_type == RIGHT_FINGER)
	{
		tb = ym + Lr + Lp;
	}
	/*else if(finger_type == MIDDLE_FINGER)
	{
	tb = ym + Lr + Lp;
	}*/
	C3 = (ta * ta + tb * tb - L1 * L1 - L2 * L2) / (2 * L1 * L2);
	if (C3 > 1.0 || C3 < -1.0)
	{
		return;
	}
	S3 = sqrt(1 - C3 * C3);
	q3 = atan2(S3, C3); //|C3|<=1
						//指付根関節
	A = L2 * C3 + L1;
	B = L2 * S3;
	C = -ym + Lr - Lp;
	if (finger_type == RIGHT_FINGER)
	{
		C = ym + Lr + Lp;
	}
	/*else if(finger_type == MIDDLE_FINGER)
	{
	C = ym + Lr + Lp;
	}*/

	if (A * A + B * B - C * C < 0.0)
	{
		return;
	}
	q2 = atan2(A, B) - atan2(sqrt(A * A + B * B - C * C), C); //A * A + B * B - C * C>=0
	if (finger_type == LEFT_FINGER)
	{
		//ref_jnt_ang[HAND02_M9] = zm;
		ref_jnt_ang[HAND_M9] = qw1;
		ref_jnt_ang[HAND_M10] = -qw1;
		ref_jnt_ang[HAND_M1] = q2;
		ref_jnt_ang[HAND_M2] = q3;
	}
	else if (finger_type == RIGHT_FINGER)
	{
		//ref_jnt_ang[HAND02_M9] = zm;
		ref_jnt_ang[HAND_M9] = qw1;
		ref_jnt_ang[HAND_M10] = -qw1;
		ref_jnt_ang[HAND_M5] = q2;
		ref_jnt_ang[HAND_M6] = q3;
	}
	else if (finger_type == MIDDLE_FINGER)
	{
		ref_jnt_ang[HAND_M9] = qw1;
		ref_jnt_ang[HAND_M10] = -qw1;
		ref_jnt_ang[HAND_M3] = q2;
		ref_jnt_ang[HAND_M4] = q3;
	}
}

//平方根の中身が負になるのでX,Yの値の調節が必要
void UturnKine(HANDJnt ref_jnt_ang, HANDJnt prepare_jnt_ang1, double TRAJ_RATE3, double time, double stime, double angle, double *camera)
{
	//逆運動学で指先に円軌道生成し指先でキューブを回す
	double X = cog_x;
	double Y = cog_y;
	double theta_s = PI / 6;//PI / 12;
	double FP = 0.0085; //人差し指半径　人差し指指先関節リンク長は0.0525
	double cube_d = 0.0555;
	double QO = cube_d / 2;
	double PO = QO / cos(theta_s);
	double FO = sqrt(FP * FP + PO * PO - 2 * FP * PO * cos(PI - theta_s));
	//double FO = FP + PO;
	double target_cog_x = 0.138; //Uturnのときの把持指のX座標 target_cog + cube_d /3 が　Uturn実行できる範囲内にある必要がある
	double offset = 0.006;
	double ys = cube_d / 2 + r - offset;
	double z = 0.07; //hand2013は0.105[m]
	double step1 = 0.1;
	double step2 = 0.20;
	double step25 = 0.25;
	double step3 = 0.30;
	double step4 = 0.40;
	double step1_t = step1;
	double step2_t = step2 - step1;
	double step25_t = step25 - step2;
	double step3_t = step3 - step25;
	double step4_t = step4 - step4;

	/*
	double step2 = 0.2;
	double step25 = 0.25;
	double step3 = 0.35;
	double step4 = 0.45;
	*/
	int jnt;

	double theta = 0.0;
	double x = 0.0;
	double y = 0.0;

	double target_y = 0.0;
	double target_x;
	//step1 準備　把持している２本の指で重心位置を奥にずらして、U回転ができるようにする
	if (time - stime < step1)
	{
		if (cog_write)
		{
			cog_x = camera[0];
			cog_y = camera[1];
			cog_write = 0;
		}
		//target_m_finger_x =  prev_mfinger_x + (0.145 + cube_d / 6 - prev_mfinger_x) * 10 * (time - stime);
		if (time - stime < step1 - 0.05)
		{
			target_cog_x = cog_x - cube_d / 3 + (target_cog_x - (cog_x - cube_d / 3)) * sin(PI * 10 * (time - stime));

			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, ys, z, LEFT_FINGER);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, -ys, z, RIGHT_FINGER);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, ys + 0.01, z + 0.030 /*0.135*/, MIDDLE_FINGER);
		}
		else
		{
			for (jnt = 0; jnt < HAND_JNT; jnt++)
				ref_jnt_ang[jnt] = prev_jnt_ang[jnt];
		}

		//ref_jnt_ang[HAND02_M3] = prev_jnt_ang[HAND02_M3];
		//ref_jnt_ang[HAND02_M4] = prev_jnt_ang[HAND02_M4];
		if (time - stime > step1 - 0.001)
		{
			cog_write = 1;
		}
	}
	//step2　回転動作 速い動作なので９０度以上回転してしまう可能性がある
	//  	 指先がキューブに接触するとキューブを飛ばしてしまうのでできるだけ90度以上に回転させる必要がある
	//		 この回転のあとyやy'の持ち替えがあり、そこでカメラからキューブの角度を取得するため、上面がずれていると角度に誤差が生まれてキャッチ動作に失敗する可能性がある
	else if (time - stime < step2)
	{
		if (cog_write)
		{
			cog_x = camera[0];
			cog_y = camera[1];
			cog_write = 0;
		}
		theta = PI / 2.0 + theta_s + sin(PI/2 * 10.0 * (time - stime - step1)) * 90 * (PI / 180);//* PI / 2.0;//20.0 * (time - stime - step1) * PI / 2.0;
		x = X + FO * cos(theta);
		y = Y + FO * sin(theta);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, x, y, z + 0.030 /*0.135*/, MIDDLE_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, ys, z, LEFT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, -ys, z, RIGHT_FINGER);
		if (time - stime > step2 - 0.001)
		{
			cog_write = 1;
		}
	}
	//指先が目標位置まで来るまで待つ　指先関節のトルクが小さいから
	else if (time - stime < step25)
	{
		for (jnt = 0; jnt < HAND_JNT; jnt++)
			ref_jnt_ang[jnt] = prev_jnt_ang[jnt];
		//指先がキューブに接触してしまうので90曲げる
		ref_jnt_ang[HAND_M4] = PI/2;//
	}
	//step3 指先を元に戻す
	else if (time - stime < step3)
	{
		target_y = cube_d / 2 + r + 0.01;
		theta = PI / 2.0 + theta_s + PI / 2.0;
		x = X + FO * cos(theta) - 0.002;
		y = Y + FO * sin(theta) + (target_y - (Y + FO * sin(theta))) * 20.0 * (time - stime - step25);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, x, y, z + 0.030 /*0.135*/, MIDDLE_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, ys, z, LEFT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, -ys, z, RIGHT_FINGER);
		//指先がキューブに接触してしまうので90曲げる
		ref_jnt_ang[HAND_M4] = PI/2;//

	}
	//step4 step1でずらした重心を元に戻す
	else if (time - stime < step4)
	{
		if (cog_write)
		{
			cog_x = camera[0];
			cog_y = camera[1];
			cog_write = 0;
		}
		target_y = cube_d / 2 + r + 0.01;
		theta = PI / 2.0 + theta_s + PI / 2.0;
		x = X + FO * cos(theta) - 0.002;
		//double x = X + FO * cos(theta) - 0.002 + 0.01 * 10 * (time - stime - step3);
		if (time - stime < step4 - 0.05)
		{
			target_x = x + (0.130 - x) * sin(PI * 10 * (time - stime - step3));
			target_cog_x = target_cog_x + (0.130 - target_cog_x) * 10 * (time - stime - step3);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_x, target_y, z + 0.030 /*0.135*/, MIDDLE_FINGER);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, ys, z, LEFT_FINGER);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, -ys, z, RIGHT_FINGER);
		}
		else
		{
			for (jnt = 0; jnt < HAND_JNT; jnt++)
				ref_jnt_ang[jnt] = prev_jnt_ang[jnt];
		}
		//ref_jnt_ang[HAND02_M3] = prev_jnt_ang[HAND02_M3];
		//ref_jnt_ang[HAND02_M4] = prev_jnt_ang[HAND02_M4];
		if (time - stime > step4 - 0.001)
		{
			cog_write = 1;
		}
	}
}
//Xturnでは重心位置の制限が強すぎたので重心位置から下へ1ブロック,手前に0.5ブロックの位置を把持して持ち上げる戦略
void Xturn2(HANDJnt ref_jnt_ang, HANDJnt prepare_jnt_ang1, double TRAJ_RATE3, double time, double stime, double angle, double *camera)
{
	double cube_d = 0.0555;			//キューブの直径WeiLong GTS2　　白いやつは　0.0565m
	double offset = 0.003;			//把持力に関係している　大きくすると強く把持してキューブが回転しないかもしれない。小さくてもキューブが把持できない。
	double xs = cog_x - cube_d / 6; //キューブの1列目と2列目の中間を持つ
	double ys = cube_d / 2 + r - offset;
	double zs = 0.07; //hnad2013は0.105[m]

	double xe = 0.130; //目標位置
	double ze = 0;
	double base_h = 0.2535;									   //土台の高さhand2013は0.28
	double base_r = 0.203;									   //土台からロボット座標の原点までの高さ hand2013は0.195
	double z = 0.070;										   //hand2013は0.105[m]
	double z_top = base_h - base_r + (2 * cube_d) / 3 + 0.004; //キューブの上層と中層の間のZ座標　0.005は少し持ち上げたほうが回転ミスが減るから
	double z_bottom = base_h - base_r + cube_d / 6;			   //一番下の把持位置 キューブの底面ブロックの真ん中のZ座標

	///step1 キューブを把持
	double step1 = 0.1;
	double step15 = 0.15;
	double step2 = 0.35;
	double step3 = 0.45;
	if (time - stime < step1)
	{
		if (cog_write)
		{
			cog_x = camera[0];
			cog_y = camera[1];
			//cog_write = 0;
		}
		if (time - stime < step1 - 0.05)
		{

			xs = xe + 0.01;	  //xs = xe + (xs - xe) * 10 * (time - stime);
			ys = ys + 3 * offset; // + 10 * 2 * offset * (time - stime);
			z = zs - (zs - z_bottom) * 10 * (time - stime);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, -ys, z, RIGHT_FINGER);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, ys, z, LEFT_FINGER);
		}
		else
		{
			xs = xe + 0.01 + (xs - xe) * 10 * (time - stime - 0.05);
			ys = ys + 3 * offset - 10 * 2 * 3 * offset * (time - stime - 0.05);
			z = zs - (zs - z_bottom) * 10 * (time - stime);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, -ys, z, RIGHT_FINGER);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, ys, z, LEFT_FINGER);
		}
		//ys = ys + 2 * offset * cos(5 * PI * (time - stime));//この時点でキューブに触れると動くので
		//z = zs - (zs - z_bottom) * sin(5 * PI * (time - stime));//sinが0.0→1.0と動く

		if (time - stime > step1 - 0.001)
		{
			cog_x = camera[0];
			cog_y = camera[1];
			cog_write = 0;
		}
	}
	//step15 一番下をちゃんと把持するまで待つ
	else if (time - stime < step15)
	{
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, -ys, z_bottom, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, ys, z_bottom, LEFT_FINGER);
	}
	//step2 キューブを持ち上げる
	else if (time - stime < step2)
	{
		//z = z_bottom + (z_top - z_bottom) * sin(2.5 * PI * (time - stime - step1));//sinが0.0→1.0
		//xs = xs + (xe - xs) * sin(2.5 * PI * (time - stime - step1));
		z = z_bottom + (z_top - z_bottom) * 5 * (time - stime - step15);
		xs = xs + (xe - xs) * 5 * (time - stime - step15);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, -ys, z, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, ys, z, LEFT_FINGER);
	}
	//step3　指先を元の位置に戻す
	else if (time - stime < step3)
	{
		//z = z_top - (z_top - z) * sin(5 * PI * (time - stime - step2));
		z = z_top - (z_top - z) * 10 * (time - stime - step2);
		//ys = ys + 2 * offset;
		//xs = xe + (xs - xe) * 2 * (time - stime - step2);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xe, -ys - 0.00, z, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xe, ys + 0.00, z, LEFT_FINGER);
		if (time - stime > step3 - 0.001)
		{
			cog_write = 1;
		}
	}
	ref_jnt_ang[HAND_M3] = prev_jnt_ang[HAND_M3];
	ref_jnt_ang[HAND_M4] = prev_jnt_ang[HAND_M4];
}

//手首回転を利用したx軸回転 x = 0.13では逆運動学がおかしくなる　x = 0.12なら大丈夫だった
void Xturn(HANDJnt ref_jnt_ang, HANDJnt prepare_jnt_ang1, double TRAJ_RATE3, double time, double stime, double angle, double cog_x, double cog_y)
{
	/*
	double TRAJ_RAD = 2 * PI * 30 / 360;
	double rate = 0;
	double DEMO1_TIME1 = 0.1;
	double TRAJ_RATE2 = 5;

	ref_jnt_ang[HAND02_M1] -= 2 * PI * 0.5 / 360;
	ref_jnt_ang[HAND02_M5] -= 2 * PI * 0.5 / 360;
	if (time - stime < 0.2)
	{
	ref_jnt_ang[HAND02_M10] = -2 * PI * 10 / 360 * (1 - cos(2.0 * PI * (time - stime)));
	}
	else if (time - stime - 0.2 < DEMO1_TIME1)
	{
	ref_jnt_ang[HAND02_M10] = TRAJ_RAD / 4 * (1.0 - cos(2.0 * PI * (time - stime) * TRAJ_RATE2)) + prepare_jnt_ang1[HAND02_M10];
	}
	else if (time - stime - 0.2 < 2 * DEMO1_TIME1)
	{
	rate = 2.0 * PI * DEMO1_TIME1 * TRAJ_RATE2;
	ref_jnt_ang[HAND02_M10] = TRAJ_RAD / 4 * (1.0 - cos(2.0 * PI * (2 * DEMO1_TIME1 - (time - stime)) * TRAJ_RATE2)) + prepare_jnt_ang1[HAND02_M10];
	}*/
	double cube_d = 0.0555; //キューブの直径WeiLong GTS2　　白いやつは　0.0565m
	double offset = 0.002;  //把持力に関係している　大きくすると強く把持してキューブが回転しないかもしれない。小さくてもキューブが把持できない。
	int use_right = 1;
	int use_left = 0;
	double xs = cog_x - cube_d / 3; //キューブ手前列の真ん中を持つ
	double ys = cube_d / 2 + r - offset;
	double zs = 0.100; //hand2013は0.105

	double xe = 0.120; //0.135;//目標位置
	double ze;

	double z = 0.100; //hand2013は0.105
	double z_top = 0.1306;
	double z_bottom = 0.0929; //一番下の把持位置
							  ///step1 キューブを把持
	double step1 = 0.1;
	double step2 = 0.3;
	double step3 = 0.4;
	if (time - stime < step1)
	{
		//ys = ys + 2 * offset * cos(5 * PI * (time - stime));//この時点でキューブに触れると動くので
		//z = zs - (zs - z_bottom) * sin(5 * PI * (time - stime));//sinが0.0→1.0と動く
		ys = ys + 2 * offset * 10 * (time - stime);
		z = zs - (zs - z_bottom) * 10 * (time - stime);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, -ys, z, use_right);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, ys, z, use_left);
	} //step2 キューブを持ち上げる
	else if (time - stime < step2)
	{
		//x=0.1340,z = 0.1145あたりでおかしい挙動
		//z = z_bottom + (z_top - z_bottom) * sin(2.5 * PI * (time - stime - step1));//sinが0.0→1.0
		//xs = xs + (xe - xs) * sin(2.5 * PI * (time - stime - step1));
		z = z_bottom + (z_top - z_bottom) * 5 * (time - stime - step1);
		xs = xs + (xe - xs) * 5 * (time - stime - step1);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, -ys, z, use_right);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, ys, z, use_left);
	} //step3　指先を元の位置に戻す
	else if (time - stime < step3)
	{
		//z = z_top - (z_top - z) * sin(5 * PI * (time - stime - step2));
		z = z_top - (z_top - z) * 10 * (time - stime - step2);
		ys = ys + 2 * offset * 10 * (time - stime - step2);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xe, -ys, z, use_right);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xe, ys, z, use_left);
	}
	//ref_jnt_ang[HAND02_M9] = z;
	//ref_jnt_ang[HAND02_M3] = xs;
	//ref_jnt_ang[HAND02_M8] = ys;
}
void Yturn2(HANDJnt ref_jnt_ang, HANDJnt prepare_jnt_ang1, double TRAJ_RATE3, double time, double stime, double angle, double *camera, int clock_wise)
{
	double rate1 = 1.0;
	//double rate2 = 1.11;
	double rate2 = 1.0;
	//double theta = 0.588; //キューブの手前列の真ん中を持つ場合
	double theta = PI / 4;
	double tan_vec_x_r = cos(-theta);
	double tan_vec_y_r = sin(-theta);
	double tan_vec_x_l = cos(PI + theta);
	double tan_vec_y_l = sin(PI + theta);
	double cube_d = 0.0555; //キューブの直径WeiLong GTS2　　白いやつは　0.0565m
	double offset = 0.006;  //把持力に関係している　大きくすると強く把持してキューブが回転しないかもしれない。小さくてもキューブが把持できない。
	double xs = 0.125;		//cog_x - cube_d / 6;//キューブ手前列の真ん中を持つ

	double ys = cube_d / 2 + r - offset;
	double z = 0.07; //0.105;
	double step1 = 0.025;
	double step2 = 0.050;
	double step3 = 0.075;

	double theta_s = 0.0;
	double theta_e = 0.0;
	double FP = 0.0; //人差し指半径　人差し指指先関節リンク長は0.0525
					 //double cube_d = 0.0;
	double QO = 0.0;
	double PO_R = 0.0; //弾く方　段々半径が小さくなっていく
	double PO_L = 0.0;
	double FO_R = 0.0;
	double FO_L = 0.0;
	double theta_f = 0.0;
	/*
	double theta_s = 0.58;
	double theta_e = PI / 6;
	double FP = 0.0085; //人差し指半径　人差し指指先関節リンク長は0.0525
	//double cube_d = 0.0555;
	double QO = cube_d / 2;
	double PO_R = QO / cos(theta_s); //弾く方　段々半径が小さくなっていく
	double PO_L = QO / cos(theta_s);
	double FO_R = sqrt(FP * FP + PO_R * PO_R - 2 * FP * PO_R * cos(PI - theta_s));
	double FO_L = sqrt(FP * FP + PO_L * PO_L - 2 * FP * PO_L * cos(PI - theta_s));
	double theta_f = theta_s - acos((FO_R * FO_R + PO_R * PO_R - FP * FP) / (2 * FO_R * PO_R));
	*/
	//ready
	if (time - stime < step1)
	{

		theta_s = 0.588;
		theta_e = PI / 9;
		FP = 0.0085; //人差し指半径　人差し指指先関節リンク長は0.0525
		cube_d = 0.0555;
		QO = cube_d / 2;
		PO_R = QO / cos(theta_s); //弾く方　段々半径が小さくなっていく
		PO_L = QO / cos(theta_s);
		FO_R = sqrt(FP * FP + PO_R * PO_R - 2 * FP * PO_R * cos(PI - theta_s));
		FO_L = sqrt(FP * FP + PO_L * PO_L - 2 * FP * PO_L * cos(PI - theta_s));
		theta_f = theta_s - acos((FO_R * FO_R + PO_R * PO_R - FP * FP) / (2 * FO_R * PO_R));

		if (cog_write)
		{
			cog_x = camera[0];
			cog_y = camera[1];
			cog_write = 0;
		}

		if (clock_wise == CLOCKWISE)
		{
			tan_vec_x_r = cog_x + FO_R * cos(-PI / 2 - theta_f);
			tan_vec_y_r = cog_y + FO_R * sin(-PI / 2 - theta_f) + offset;
			tan_vec_x_l = cog_x + FO_L * cos(PI / 2 + theta_f);
			tan_vec_y_l = cog_y + FO_L * sin(PI / 2 + theta_f) - offset;
		}
		else if (clock_wise == COUNTER_CLOCKWISE)
		{
			tan_vec_x_r = cog_x + FO_L * cos(-PI / 2 - theta_f);
			tan_vec_y_r = cog_y + FO_L * sin(-PI / 2 - theta_f) + offset;
			tan_vec_x_l = cog_x + FO_R * cos(PI / 2 + theta_f);
			tan_vec_y_l = cog_y + FO_R * sin(PI / 2 + theta_f) - offset;
		}
		//準備でキューブ手前ブロックの真ん中を把持する　offset分だけ指先を押し込んでいる
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, cog_x - cube_d/3, -ys, z, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, cog_x - cube_d/3, ys, z, LEFT_FINGER);

		/*
		ref_jnt_ang[HAND02_M1] = prev_jnt_ang[HAND02_M1];
		ref_jnt_ang[HAND02_M5] = prev_jnt_ang[HAND02_M5];
		ref_jnt_ang[HAND02_M2] = prev_jnt_ang[HAND02_M2];
		ref_jnt_ang[HAND02_M6] = prev_jnt_ang[HAND02_M6];
		*/
		/*
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, -ys, z, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, ys, z, LEFT_FINGER);
		*/
		if (time - stime > step1 - 0.001)
		{
			cog_write = 1;
		}
	}
	//throw
	else if (time - stime < step2)
	{

		theta_s = 0.58;
		theta_e = PI / 6;
		FP = 0.0085; //人差し指半径　人差し指指先関節リンク長は0.0525
		cube_d = 0.0555;
		QO = cube_d / 2;
		//double PO_R = QO / cos(theta_s - theta_e * 10 * (time - stime - step1)); //弾く方　段々半径が小さくなっていく
		//double PO_L = QO / cos(theta_s + theta_e * 10 * (time - stime - step1));
		PO_R = QO / cos(theta_s); //弾く方　段々半径が小さくなっていく
		PO_L = QO / cos(theta_s);
		FO_R = sqrt(FP * FP + PO_R * PO_R - 2 * FP * PO_R * cos(PI - theta_s));
		FO_L = sqrt(FP * FP + PO_L * PO_L - 2 * FP * PO_L * cos(PI - theta_s));
		theta_f = theta_s - acos((FO_R * FO_R + PO_R * PO_R - FP * FP) / (2 * FO_R * PO_R));
		/*
		if (cog_write)
		{
		cog_x = camera[1];
		cog_y = camera[2];
		cog_write = 0;
		}
		*/
		// 	指先だけ回転

		if (clock_wise == CLOCKWISE)
		{
			tan_vec_x_r = cog_x + FO_R * cos(-PI / 2 - theta_f);
			tan_vec_y_r = cog_y + FO_R * sin(-PI / 2 - theta_f) + offset;
			tan_vec_x_l = cog_x + FO_L * cos(PI / 2 + theta_f);
			tan_vec_y_l = cog_y + FO_L * sin(PI / 2 + theta_f) - offset;
		}
		else if (clock_wise == COUNTER_CLOCKWISE)
		{
			tan_vec_x_r = cog_x + FO_L * cos(-PI / 2 - theta_f);
			tan_vec_y_r = cog_y + FO_L * sin(-PI / 2 - theta_f) + offset;
			tan_vec_x_l = cog_x + FO_R * cos(PI / 2 + theta_f);
			tan_vec_y_l = cog_y + FO_R * sin(PI / 2 + theta_f) - offset;
		}

		//円軌道
		/*
		if (clock_wise == CLOCKWISE)
		{
		tan_vec_x_r = cog_x + FO_R * cos(-PI / 2 - theta_f - theta_e * 20 * (time - stime - step1));
		tan_vec_y_r = cog_y + FO_R * sin(-PI / 2 - theta_f - theta_e * 20 * (time - stime - step1)) + offset;;
		tan_vec_x_l = cog_x + FO_L * cos(PI / 2 + theta_f - theta_e * 20 * (time - stime - step1));
		tan_vec_y_l = cog_y + FO_L * sin(PI / 2 + theta_f - theta_e * 20 * (time - stime - step1)) - offset;;
		}
		else if (clock_wise == COUNTER_CLOCKWISE)
		{
		tan_vec_x_r = cog_x + FO_L * cos(-PI / 2 - theta_f + theta_e * 20 * (time - stime - step1));
		tan_vec_y_r = cog_y + FO_L * sin(-PI / 2 - theta_f + theta_e * 20 * (time - stime - step1)) + offset;;
		tan_vec_x_l = cog_x + FO_R * cos(PI / 2 + theta_f + theta_e * 20 * (time - stime - step1));
		tan_vec_y_l = cog_y + FO_R * sin(PI / 2 + theta_f + theta_e * 20 * (time - stime - step1)) - offset;;
		}
		*/
		//invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, tan_vec_x_r, tan_vec_y_r, z, RIGHT_FINGER);
		//invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, tan_vec_x_l, tan_vec_y_l, z, LEFT_FINGER);
		//指先を回転
		//ref_jnt_ang[HAND02_M1] = prev_jnt_ang[HAND02_M1];
		//ref_jnt_ang[HAND02_M5] = prev_jnt_ang[HAND02_M5];
		if (clock_wise == CLOCKWISE)
		{
			ref_jnt_ang[HAND_M6] += -PI / 5 * (1.0 - cos(2.0 * PI * (time - stime - step1) * 10));
			ref_jnt_ang[HAND_M2] += PI / 5 * (1.0 - cos(2.0 * PI * (time - stime - step1) * 10));
		}
		else if (clock_wise == COUNTER_CLOCKWISE)
		{
			ref_jnt_ang[HAND_M6] += PI / 5 * (1.0 - cos(2.0 * PI * (time - stime - step1) * 10));
			ref_jnt_ang[HAND_M2] += -PI / 5 * (1.0 - cos(2.0 * PI * (time - stime - step1) * 10));
		}

		/*tan_vec_x_r = cos(PI - theta_f);
		tan_vec_y_r = sin(PI - theta_f);
		tan_vec_x_l = cos(theta_f);
		tan_vec_y_l = sin(theta_f);
		*/
		/*
		if(clock_wise == CLOCKWISE)
		{
		tan_vec_x_r = -rate1 * 0.015 / 0.025 * (time - stime - step1) * tan_vec_x_r;
		tan_vec_y_r = -rate1 * 0.015 / 0.025 * (time - stime - step1) * tan_vec_y_r;
		tan_vec_x_l = -rate2 * 0.015 / 0.025 * (time - stime - step1) * tan_vec_x_l;
		tan_vec_y_l = -rate2 * 0.015 / 0.025 * (time - stime - step1) * tan_vec_y_l;
		}
		else if(clock_wise == COUNTER_CLOCKWISE)
		{
		tan_vec_x_r = rate2 * 0.015 / 0.025 * (time - stime - step1) * tan_vec_x_r;
		tan_vec_y_r = rate2 * 0.015 / 0.025 * (time - stime - step1) * tan_vec_y_r;
		tan_vec_x_l = rate1 * 0.015 / 0.025 * (time - stime - step1) * tan_vec_x_l;
		tan_vec_y_l = rate1 * 0.015 / 0.025 * (time - stime - step1) * tan_vec_y_l;
		}
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs + tan_vec_x_r, -ys + tan_vec_y_r, z, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs + tan_vec_x_l, ys + tan_vec_y_l, z, LEFT_FINGER);
		*/
		if (time - stime > step2 - 0.001)
		{
			cog_write = 1;
		}
	}
	//wait
	else if (time - stime < step3)
	{

		theta_s = 0.58;
		theta_e = PI / 6;
		FP = 0.0085; //人差し指半径　人差し指指先関節リンク長は0.0525
		cube_d = 0.0555;
		QO = cube_d / 2;
		//double PO_R = QO / cos(theta_s - theta_e); //弾く方　段々半径が小さくなっていく
		//double PO_L = QO / cos(theta_s + theta_e);
		PO_R = QO / cos(theta_s); //弾く方　段々半径が小さくなっていく
		PO_L = QO / cos(theta_s);
		FO_R = sqrt(FP * FP + PO_R * PO_R - 2 * FP * PO_R * cos(PI - theta_s));
		FO_L = sqrt(FP * FP + PO_L * PO_L - 2 * FP * PO_L * cos(PI - theta_s));
		theta_f = theta_s - acos((FO_R * FO_R + PO_R * PO_R - FP * FP) / (2 * FO_R * PO_R));

		/*
		if (clock_wise == CLOCKWISE)
		{
		tan_vec_x_r = cog_x + FO_R * cos(-PI / 2 - theta_f - theta_e);
		tan_vec_y_r = cog_y + FO_R * sin(-PI / 2 - theta_f - theta_e);
		tan_vec_x_l = cog_x + FO_L * cos(PI / 2 + theta_f - theta_e);
		tan_vec_y_l = cog_y + FO_L * sin(PI / 2 + theta_f - theta_e);
		}
		else if (clock_wise == COUNTER_CLOCKWISE)
		{
		tan_vec_x_r = cog_x + FO_L * cos(-PI / 2 - theta_f + theta_e);
		tan_vec_y_r = cog_y + FO_L * sin(-PI / 2 - theta_f + theta_e);
		tan_vec_x_l = cog_x + FO_R * cos(PI / 2 + theta_f + theta_e);
		tan_vec_y_l = cog_y + FO_R * sin(PI / 2 + theta_f + theta_e);
		}
		*/

		if (clock_wise == CLOCKWISE)
		{
			tan_vec_x_r = cog_x + FO_R * cos(-PI / 2 - theta_f);
			tan_vec_y_r = cog_y + FO_R * sin(-PI / 2 - theta_f) + offset;
			tan_vec_x_l = cog_x + FO_L * cos(PI / 2 + theta_f);
			tan_vec_y_l = cog_y + FO_L * sin(PI / 2 + theta_f) - offset;
		}
		else if (clock_wise == COUNTER_CLOCKWISE)
		{
			tan_vec_x_r = cog_x + FO_L * cos(-PI / 2 - theta_f);
			tan_vec_y_r = cog_y + FO_L * sin(-PI / 2 - theta_f) + offset;
			tan_vec_x_l = cog_x + FO_R * cos(PI / 2 + theta_f);
			tan_vec_y_l = cog_y + FO_R * sin(PI / 2 + theta_f) - offset;
		}

		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, tan_vec_x_r, tan_vec_y_r, z, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, tan_vec_x_l, tan_vec_y_l, z, LEFT_FINGER);

		/*
		tan_vec_x_r = cos(PI - theta_f);
		tan_vec_y_r = sin(PI - theta_f);
		tan_vec_x_l = cos(theta_f);
		tan_vec_y_l = sin(theta_f);
		*/
		/*
		if(clock_wise == CLOCKWISE)
		{
		tan_vec_x_r = -rate1 * 0.015 * tan_vec_x_r;
		tan_vec_y_r = -rate1 * 0.015 * tan_vec_y_r;
		tan_vec_x_l = -rate2 * 0.015 * tan_vec_x_l;
		tan_vec_y_l = -rate2 * 0.015 * tan_vec_y_l;
		}
		else if(clock_wise == COUNTER_CLOCKWISE)
		{
		tan_vec_x_r = rate2 * 0.015 * tan_vec_x_r;
		tan_vec_y_r = rate2 * 0.015 * tan_vec_y_r;
		tan_vec_x_l = rate1 * 0.015 * tan_vec_x_l;
		tan_vec_y_l = rate1 * 0.015 * tan_vec_y_l;
		}
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs + tan_vec_x_r, -ys + tan_vec_y_r, z, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs + tan_vec_x_l, ys + tan_vec_y_l, z, LEFT_FINGER);
		*/
		if (clock_wise == CLOCKWISE)
		{
			ref_jnt_ang[HAND_M1] = -PI / 12;
			ref_jnt_ang[HAND_M5] = -PI / 12;
		}
		else if (clock_wise == COUNTER_CLOCKWISE)
		{
			ref_jnt_ang[HAND_M1] = -PI / 12;
			ref_jnt_ang[HAND_M5] = -PI / 12;
		}
	}
	//Catch
	else if (angle > 70.0 || angle < 20.0)
	{
		tan_vec_x_r = 0;// * tan_vec_x_r;
		tan_vec_y_r = 0;// * tan_vec_y_r;
		tan_vec_x_l = 0;// * tan_vec_x_l;
		tan_vec_y_l = 0;// * tan_vec_y_l;
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs + cube_d / 6 , -ys, z, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs + cube_d / 6 , ys, z, LEFT_FINGER);
	}
	else
	{
		if (clock_wise == CLOCKWISE)
		{
			ref_jnt_ang[HAND_M1] = -PI / 12;
			ref_jnt_ang[HAND_M2] = PI / 4; //prev_jnt_ang[HAND02_M2];
			ref_jnt_ang[HAND_M5] = -PI / 12;
			ref_jnt_ang[HAND_M6] = PI / 4; //prev_jnt_ang[HAND02_M6];
		}
		else if (clock_wise == COUNTER_CLOCKWISE)
		{
			ref_jnt_ang[HAND_M1] = -PI / 12;
			ref_jnt_ang[HAND_M2] = PI / 4; //prev_jnt_ang[HAND02_M2];
			ref_jnt_ang[HAND_M5] = -PI / 12;
			ref_jnt_ang[HAND_M6] = PI / 4; //prev_jnt_ang[HAND02_M6];
		}
	}
	ref_jnt_ang[HAND_M3] = -PI / 6;
	ref_jnt_ang[HAND_M4] = prev_jnt_ang[HAND_M4];
}

//指先を利用したy軸回転
void Yturn(HANDJnt ref_jnt_ang, HANDJnt prepare_jnt_ang1, double TRAJ_RATE3, double time, double stime, double angle)
{

	//throw
	if (time - stime < 0.025)
	{
		ref_jnt_ang[HAND_M6] = PI / 90 * 10 * (1.0 - cos(2.0 * PI * (time - stime) * TRAJ_RATE3)) + prepare_jnt_ang1[HAND_M2];
		ref_jnt_ang[HAND_M2] = -PI / 4 * (1.0 - cos(2.0 * PI * (time - stime) * TRAJ_RATE3)) + prepare_jnt_ang1[HAND_M6];
	}
	else if (time - stime < 0.05)
	{
		ref_jnt_ang[HAND_M1] = -PI / 25 * (1.0 - cos(2.0 * PI * (time - stime) * TRAJ_RATE3)) + prepare_jnt_ang1[HAND_M1];
		ref_jnt_ang[HAND_M5] = -PI / 25 * (1.0 - cos(2.0 * PI * (time - stime) * TRAJ_RATE3)) + prepare_jnt_ang1[HAND_M5];
	}
	//Catch
	else if (angle > 70.0 || angle < 20.0)
	{
		ref_jnt_ang[HAND_M1] = prepare_jnt_ang1[HAND_M1];
		ref_jnt_ang[HAND_M5] = prepare_jnt_ang1[HAND_M5];
	}
	else
	{
		ref_jnt_ang[HAND_M1] = -PI / 25 * (1.0 - cos(2.0 * PI * 0.05 * TRAJ_RATE3)) + prepare_jnt_ang1[HAND_M1];
		ref_jnt_ang[HAND_M5] = -PI / 25 * (1.0 - cos(2.0 * PI * 0.05 * TRAJ_RATE3)) + prepare_jnt_ang1[HAND_M5];
	}
}

//回転記号の配列を受け取り順番に実行する
void execFromString(HANDJnt ref_jnt_ang, HANDJnt prepare_jnt_ang, double TRAJ_RATE3, double time, double *camData, int *operation)
{
	//const int opNum = sizeof operation / sizeof operation[0];
	int opNum = 4;
	//operation[n]がop_time[n]の時間に終了する
	double op_time[opNum];
	int n;
	op_time[0] = 2.0;
	for (n = 0; n < opNum; n++)
	{
		if (operation[n] == U_TURN_P)
		{
			if (n == 0)
			{
				op_time[n] += TIME_U_P;
			}
			else
			{
				op_time[n] = op_time[n - 1] + TIME_U_P;
			}
		}
		else if (operation[n] == X_ROTATION)
		{
			if (n == 0)
			{
				op_time[n] += TIME_X;
			}
			else
			{
				op_time[n] = op_time[n - 1] + TIME_X;
			}
		}
		else if (operation[n] == Y_ROTATION)
		{
			if (n == 0)
			{
				op_time[n] += TIME_Y;
			}
			else
			{
				op_time[n] = op_time[n - 1] + TIME_Y;
			}
		}
		else if (operation[n] == Y_ROTATION_P)
		{
			if (n == 0)
			{
				op_time[n] += TIME_Y;
			}
			else
			{
				op_time[n] = op_time[n - 1] + TIME_Y;
			}
		}
	}

	//キューブ操作実行
	for (n = 0; n < opNum; n++)
	{
		if (time < 2.0)
			break;
		if (time < op_time[n])
		{
			if (operation[n] == U_TURN_P)
			{
				UturnKine(ref_jnt_ang, prepare_jnt_ang, TRAJ_RATE3, time, op_time[n] - TIME_U_P, camData[2], camData);
			}
			else if (operation[n] == X_ROTATION)
			{
				Xturn2(ref_jnt_ang, prepare_jnt_ang, TRAJ_RATE3, time, op_time[n] - TIME_X, camData[2], camData);
			}
			else if (operation[n] == Y_ROTATION)
			{
				Yturn2(ref_jnt_ang, prepare_jnt_ang, TRAJ_RATE3, time, op_time[n] - TIME_Y, camData[2], camData, CLOCKWISE);
			}
			else if (operation[n] == Y_ROTATION_P)
			{
				Yturn2(ref_jnt_ang, prepare_jnt_ang, TRAJ_RATE3, time, op_time[n] - TIME_Y, camData[2], camData, COUNTER_CLOCKWISE);
			}
			//一度実行したらfor文から抜ける
			break;
		}
	}
}

// 準備姿勢

//static HANDJnt	prepare_jnt_ang = {-0.270, 1.090, 0.0 ,-PI/2.0 ,-0.270, 1.090  ,-PI/2.0 ,PI/2.0 , 0.0, 0.0 , 0.0}; //52mm

//static HANDJnt prepare_jnt_ang = { 0.0, 0.0,	   //左指
//0.0, 0.0,	   //中指
//0.0, 0.0,	   //右指
//-1.006147, 0.0, //左指旋回　右指旋回
//0.0, 0.0, 0.0 };	  //手首旋回(9,10)　手首屈曲

static HANDJnt prepare_jnt_ang = {0.0, 0.0,		  //左指
								  0.0, 0.0,		  //中指
								  0.0, 0.0,		  //右指
								  0.0, 0.0,		  //左指旋回　右指旋回
								  0.0, 0.0, 0.0}; //手首旋回(9,10)　手首屈曲
//////////////////////////////////////////////////////
double trajGeneLine(const double motion_time, // 関数開始からの時間
					const double all_time,	// 全体の時間
					const double start_jnt,   // 初期関節角
					const double end_jnt)	 // 終端関節角
{
	return start_jnt + motion_time * (end_jnt - start_jnt) / all_time;
}

//////////////////////////////////////////////////////
double handWaveGeneSin(const double motion_time, // 波動関数開始からの時間
					   const double all_time,	// 全体の時間
					   const double start_jnt,   // 初期関節角
					   const double end_jnt)	 // 終端関節角
{
	return start_jnt + (end_jnt - start_jnt) * (1 - cos(PI * motion_time / all_time)) / 2;
}

//////////////////////////////////////////////////////
int handTrajApp(HANDJnt ref_jnt_ang, DATA *data, double time, double *camData)
{
#if 1
#define TRAJ_RAD PI / 4.0
#define TRAJ_RATE1 0.1
#define TRAJ_RATE2 1.0
#define TRAJ_RATE3 10.0

#define DEMO1_TIME1 0.5
#define DEMO1_TIME2 8.0
#define DEMO1_TIME3 9.0
	int jnt;
	double rate = 0.0;

	double cube_d = 0.0; //キューブの直径WeiLong GTS2　　白いやつは　0.0565m
	double offset = 0.0; //把持力に関係している　大きくすると強く把持してキューブが回転しないかもしれない。小さくてもキューブが把持できない。
	double xs = 0.0;	 //キューブ手前列の真ん中を持つ
	double ys = 0.0;	 //- offset;// - 0.004; //+ 2 * offset;
	double zs = 0.0;

	double interval = 0.0;

	int i = 0;
	//data[0] = camData[0];
	for (jnt = 0; jnt < HAND_JNT; jnt++)
		ref_jnt_ang[jnt] = prepare_jnt_ang[jnt];
	//前回(0.001秒前)の値を最初に入れておいてその後変更する
	//逆運動学に解がなかった場合に角度が0になってしまうことを防止するため
	for (jnt = 0; jnt < HAND_JNT; jnt++)
		ref_jnt_ang[jnt] = prev_jnt_ang[jnt];
	//デバック用
	/*
	double cube_d = 0.0555;
	double x = camera[1];
	double y = cube_d / 2 + r + 0.005;
	double z = 0.105;
	invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, x, -y, z, 1);
	invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, x, y, z, 0);
	*/

	//初期位置へ移動
	if (time < 2.0)
	{
		if (time < 0.5)
		{
			cog_x = camData[0];
			cog_y = camData[1];
		}
		cube_d = 0.0555;		 //キューブの直径WeiLong GTS2　　白いやつは　0.0565m
		offset = 0.005;			 //把持力に関係している　大きくすると強く把持してキューブが回転しないかもしれない。小さくてもキューブが把持できない。
		xs = cog_x - cube_d / 3; //キューブ手前列の真ん中を持つ
		ys = cube_d / 2 + r;	 //- offset;// - 0.004; //+ 2 * offset;
		zs = 0.07;				 //hand2013は0.105[m]
		prev_mfinger_x = cog_x - cube_d / 3;
		prev_mfinger_y = cube_d / 2 + r + 0.002;
		invKine(ref_jnt_ang, prepare_jnt_ang, time, 0.0, 0.0, xs, -ys, zs, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang, time, 0.0, 0.0, xs, ys, zs, LEFT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang, time, 0.0, 0.0, xs, prev_mfinger_y, zs + 0.030 /*0.135*/, MIDDLE_FINGER);

		ref_jnt_ang[HAND_M1] *= (0.5 * time);
		ref_jnt_ang[HAND_M2] *= (0.5 * time);
		ref_jnt_ang[HAND_M3] *= (0.5 * time);
		ref_jnt_ang[HAND_M4] *= (0.5 * time);
		ref_jnt_ang[HAND_M5] *= (0.5 * time);
		ref_jnt_ang[HAND_M6] *= (0.5 * time);
		ref_jnt_ang[HAND_M9] *= (0.5 * time);
		ref_jnt_ang[HAND_M10] *= (-0.5 * time);
	}
	//	行いたい操作を順番に記号で入れる配列
	//int operation[] = {U_TURN_P, Y_ROTATION, X_ROTATION, U_TURN_P};
	int operation[] = {Y_ROTATION, X_ROTATION, X_ROTATION, U_TURN_P};
	execFromString(ref_jnt_ang, prepare_jnt_ang, TRAJ_RATE3, time , camData, operation);
	//手首を利用した回転　逆運動学利用ver
	/*
	for(int i = 1; i <= 10 ; i++){
	double interval = 0.6;
	if(time < 2.0) break;
	if(time > 2.0 + interval * (i - 1) && time < 2.0 + interval * i - 0.2)
	{
	if(cog_write)
	{
	cog_x = camera[1];
	cog_y = camera[2];
	cog_write = 0;
	}
	Xturn2(ref_jnt_ang, prepare_jnt_ang1, TRAJ_RATE3, time, 2.0 + interval * (i - 1), camera[3], cog_x , cog_y);
	if(time > 2.0 + interval * i - 0.002 - 0.2)
	{
	cog_write = 1;
	}
	break;
	}
	else if(time < 2.0 + interval * i)
	{
	Yturn2(ref_jnt_ang, prepare_jnt_ang1, TRAJ_RATE3, time, 2.0 + interval * (i - 1) + 0.4 , camera[3], cog_x , cog_y);
	break;
	}
	//ref_jnt_ang[HAND02_M9] = cog_x;
	}
	*/
	// U面回転　連続動作 成功2018/02/16

	/*
	for (i = 1; i <= 10; i++)
	{
		interval = 1.0; //0.35 + 0.2 + 0.45
		if (time < 2.0)
			break;
		if (time < 2.0 + interval * i - 0.65) //0.45s
		{
			UturnKine(ref_jnt_ang, prepare_jnt_ang, TRAJ_RATE3, time, 2.0 + interval * (i - 1), camData[2], camData);
			break;
		}
		else if (time < 2.0 + interval * i - 0.45) //0.2s
		{
			if (i % 2 == 0)
			{
				Yturn2(ref_jnt_ang, prepare_jnt_ang, TRAJ_RATE3, time, 2.0 + interval * (i - 1) + 0.35, camData[2], camData, CLOCKWISE);
			}
			else
			{
				Yturn2(ref_jnt_ang, prepare_jnt_ang, TRAJ_RATE3, time, 2.0 + interval * (i - 1) + 0.35, camData[2], camData, COUNTER_CLOCKWISE);
			}

			//Yturn(ref_jnt_ang, prepare_jnt_ang1, TRAJ_RATE3, time, 2.0 + interval * (i - 1) + 0.45, camera[3]);
			break;
		}
		else if (time < 2.0 + interval * i) //0.45s
		{
			Xturn2(ref_jnt_ang, prepare_jnt_ang, TRAJ_RATE3, time, 2.0 + interval * (i - 1) + 0.55, camData[2], camData);
			break;
		}
	}
	*/


	//Y回転
	/*
	if (time > 2.0 && time < 10.0)
	{
	Yturn2(ref_jnt_ang, prepare_jnt_ang1, TRAJ_RATE3, time, 2.0, camera[3], cog_x, cog_y, CLOCKWISE);
	}
	*/
	//関数版
	/*if(time < 2.0)
	{
	return 0;
	}
	else if (time < 2.2)
	{
	Yturn(ref_jnt_ang, prepare_jnt_ang1, TRAJ_RATE3, time, 2.0, camera[3]);
	}
	else if (time < 2.7)
	{
	Xturn(ref_jnt_ang, prepare_jnt_ang1, TRAJ_RATE3, time, 2.2, camera[3]);
	}
	else if (time < 4.0)
	{
	Uturn(ref_jnt_ang, prepare_jnt_ang1, TRAJ_RATE3, time, 2.7, camera[3]);
	}*/

	//Uturn(ref_jnt_ang, prepare_jnt_ang1, TRAJ_RATE3, time, camera[3]);
	//関節角を保存
	for (jnt = 0; jnt < HAND_JNT; jnt++)
		prev_jnt_ang[jnt] = ref_jnt_ang[jnt];
	return 0;
#else
#define TRAJ_RAD PI / 4.0
#define TRAJ_RATE1 0.1
#define TRAJ_RATE2 1.0
#define TRAJ_RATE3 10.0

#define DEMO1_TIME1 5.0
#define DEMO1_TIME2 8.0
#define DEMO1_TIME3 9.0
	int jnt;
	double rate = 0.0;
	for (jnt = 0; jnt < HAND_JNT; jnt++)
		ref_jnt_ang[jnt] = prepare_jnt_ang[jnt];
	ref_jnt_ang[HAND_M1] = -TRAJ_RAD + prepare_jnt_ang[HAND_M1];
	ref_jnt_ang[HAND_M3] = -TRAJ_RAD + prepare_jnt_ang[HAND_M3];
	ref_jnt_ang[HAND_M5] = -TRAJ_RAD + prepare_jnt_ang[HAND_M5];
	//ref_jnt_ang[HAND_M2] = TRAJ_RAD + prepare_jnt_ang[HAND_M2];
	//ref_jnt_ang[HAND_M4] = TRAJ_RAD + prepare_jnt_ang[HAND_M4];
	//ref_jnt_ang[HAND_M6] = TRAJ_RAD + prepare_jnt_ang[HAND_M6];
	if (time < DEMO1_TIME1)
	{
		//ref_jnt_ang[HAND_M1] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M1];
		//ref_jnt_ang[HAND_M3] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M3];
		//ref_jnt_ang[HAND_M5] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M5];
		//ref_jnt_ang[HAND_M2] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M2];
		//ref_jnt_ang[HAND_M4] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M4];
		//ref_jnt_ang[HAND_M6] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M6];
		//ref_jnt_ang[HAND_M7] = PI/3;

		//ref_jnt_ang[HAND_M7] = -TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE1)) + prepare_jnt_ang[HAND_M7];
		//ref_jnt_ang[HAND_M8] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE1)) + prepare_jnt_ang[HAND_M8];

		//ref_jnt_ang[HAND_M9] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M9];
		//ref_jnt_ang[HAND_M10] = -TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M10];

		//ref_jnt_ang[HAND_M11] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE1)) + prepare_jnt_ang[HAND_M11];
	}
	else if (time < DEMO1_TIME2)
	{
		rate = 2.0 * PI * DEMO1_TIME1 * TRAJ_RATE1;
		/*
		ref_jnt_ang[HAND_M1] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M1];
		ref_jnt_ang[HAND_M3] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M3];
		ref_jnt_ang[HAND_M5] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M5];
		ref_jnt_ang[HAND_M2] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M2];
		ref_jnt_ang[HAND_M4] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M4];
		ref_jnt_ang[HAND_M6] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M6];
		*/

		//ref_jnt_ang[HAND_M9] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M9];
		//ref_jnt_ang[HAND_M10] = -TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M10];
	}
	/*
	else if (time <DEMO1_TIME3) {
	rate = 2.0*PI*DEMO1_TIME2*TRAJ_RATE2;
	ref_jnt_ang[HAND_M1] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE3 - rate)) + prepare_jnt_ang[HAND_M1];
	ref_jnt_ang[HAND_M3] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE3 - rate)) + prepare_jnt_ang[HAND_M3];
	ref_jnt_ang[HAND_M5] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE3 - rate)) + prepare_jnt_ang[HAND_M5];
	ref_jnt_ang[HAND_M2] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE3 - rate)) + prepare_jnt_ang[HAND_M2];
	ref_jnt_ang[HAND_M4] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE3 - rate)) + prepare_jnt_ang[HAND_M4];
	ref_jnt_ang[HAND_M6] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE3 - rate)) + prepare_jnt_ang[HAND_M6];
	}
	*/
	else
	{
		//ref_jnt_ang[HAND_M2] = 0.0;
		//ref_jnt_ang[HAND_M4] = 0.0;
		//ref_jnt_ang[HAND_M6] = 0.0;
		//ref_jnt_ang[HAND_M1] = 0.0;
		//ref_jnt_ang[HAND_M3] = 0.0;
		//ref_jnt_ang[HAND_M5] = 0.0;
		for (jnt = 0; jnt < HAND_JNT; jnt++)
			ref_jnt_ang[jnt] = prepare_jnt_ang[jnt];
	}

	return 0;


#endif
}

//////////////////////////////////////
// ハンド初期設定
//////////////////////////////////////
int handAppSet(HAND *hand)
{
	int jnt;
	//////////////////////////////////////
	hand->cons.da_limit = 1.0; // 0.0 ～ 1.0（この値にDA_LIMIT_HANDをかけたDA指令を出力）
							   //////////////////////////////////////
	hand->base_homo = &hand_base_homo;
	hand->ctrl_coef = &hand_ctrl_coef;
	hand->init_jnt_ang = init_jnt_ang;
	hand->prepare_jnt_ang = prepare_jnt_ang;
	return 0;
}

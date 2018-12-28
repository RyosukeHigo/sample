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
// HANDCtrlCoef hand_ctrl_coef = {{15.0, 20.0, 15.0, 20.0, 15.0, 20.0, 22.0, 22.0, 19.0, 20.0, 14.0}, // Kp
// 							   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},			   // Ti
// 							   {60.0, 75.0, 50.0, 75.0, 60.0, 75.0, 70.0, 70.0, 36.0, 40.0, 26.0}, // Td
// 							   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},			   // Cf
// 							   1.0};

//指先は0.07秒(70ms)で90度回転
//20181012 手首変更             左指		中指		右指　　　　左旋回右旋回
//2018/12/19 中指と左指を入れ替え
HANDCtrlCoef hand_ctrl_coef = {{15.0, 20.0, 15.0, 20.0, 15.0, 20.0, 22.0, 22.0, 40.0, 20.0, 14.0},  // Kp
							   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},				// Ti
							   {40.0, 75.0, 60.0, 75.0, 60.0, 75.0, 70.0, 70.0, 140.0, 40.0, 26.0}, // Td
							   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},				// Cf
							   1.0};

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

#define THETA_S (PI / 6) //PI / 12;
#define QO (g_cube_d / 2)
#define PO (QO / cos(THETA_S))
#define FP 0.0085 //人差し指半径　人差し指指先関節リンク長は0.0525
#define FO sqrt(FP *FP + PO * PO - 2 * FP * PO * cos(PI - THETA_S))

const double table_h = 0.316;	//黒い昇降テーブルの高さ　この上にルービックキューブを乗せる
const double Lb = 0.003 + 0.225; //ロボット固定台から手首屈曲関節までの高さ　[固定板の厚み＋固定台から手首屈曲までの高さ]で求める
								 //指付け根リンク長 //古い方のハンドは0.062[m] hand2013のやつは0.0639[m]
								 //const double L2 = 0.0365;//指先リンク長　半球の中心を指先とする場合は0.0365m　指先の場合0.045m
								 //const double L2 = 0.049; //指サックつけたとき
const double r = 0.045 - 0.0365; //指先半球の半径
const double g_Lw0 = 0.110;
const double g_cube_d = 0.0555; //使用するルービックキューブの直径

//重心位置をグローバルで保存cog_writeで書き込み制限
double cog_x = 0.0;
double cog_y = 0.0;
int cog_write = 1;

//前のハンド関節の値を保存
double prev_jnt_ang[HAND_JNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

double prev_mfinger_x;
double prev_mfinger_y;
double prev_lfinger_x;
double prev_lfinger_y;
double prev_rfinger_x;
double prev_rfinger_y;
//一時的な関節角保存配列
double tmp_jnt_ang[HAND_JNT];
//逆運動学で右指、左指の角度を求める
//並木先生のキネマティクスのドキュメントとはハンド屈曲関節の回転方向の定義が逆だったのでqw1の符号を反転させた
//解がないときは前の値を代入するようにした
void invKine(HANDJnt ref_jnt_ang, HANDJnt prepare_jnt_ang1, double time, double stime, double angle, double xm, double ym, double zm, int finger_type)
{
	//手首屈曲関節
	double Lr = 0.030;
	double L1 = 0.062;  //指先根元リンク
	double L2 = 0.0365; //指先リンク
	double Lw0 = g_Lw0; //手首の長さ
	double Lw1 = 0.045; //手首から指根本までの長さ
	double Lpr = -0.03;
	double Lpl = 0.03;
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
	if (finger_type == MIDDLE_FINGER)
	{
		Lw0 += Lr;
	}
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
	tb = ym - Lr - Lp;
	if (finger_type == RIGHT_FINGER)
	{
		tb = ym + Lr - Lp;
	}

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
	C = -ym + Lr + Lp;
	if (finger_type == RIGHT_FINGER)
	{
		C = ym + Lr - Lp;
	}

	if (A * A + B * B - C * C < 0.0)
	{
		return;
	}
	q2 = atan2(A, B) - atan2(sqrt(A * A + B * B - C * C), C); //A * A + B * B - C * C>=0
	if (finger_type == LEFT_FINGER)
	{
		ref_jnt_ang[HAND_M9] = -qw1;
		ref_jnt_ang[HAND_M1] = q2;
		ref_jnt_ang[HAND_M2] = q3;
	}
	else if (finger_type == RIGHT_FINGER)
	{
		ref_jnt_ang[HAND_M9] = -qw1;
		ref_jnt_ang[HAND_M5] = q2;
		ref_jnt_ang[HAND_M6] = q3;
	}
	else if (finger_type == MIDDLE_FINGER)
	{
		ref_jnt_ang[HAND_M9] = -qw1;
		ref_jnt_ang[HAND_M3] = q2;
		ref_jnt_ang[HAND_M4] = q3;
	}
}

// U'を回す
void UturnKine(HANDJnt ref_jnt_ang, HANDJnt prepare_jnt_ang1, double TRAJ_RATE3, double time, double stime, double angle, double *camera)
{
	//逆運動学で指先に円軌道生成し指先でキューブを回す
	double X = cog_x;
	double Y = cog_y;
	//double FO = g_FP + PO;
	double target_cog_x = 0.138; //Uturnのときの把持指のX座標 target_cog + g_cube_d /3 が　Uturn実行できる範囲内にある必要がある
	double x_f_end = 0.12;		 //0.1265;
	double offset = 0.008;
	double ys = g_cube_d / 2 + r - offset;
	double z = g_Lw0; //hand2013は0.105[m]
	double step1 = 0.10;
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

		theta = PI / 2.0 + THETA_S;
		x = camera[0] + FO * cos(theta);
		y = camera[1] + FO * sin(theta);
		//target_m_finger_x =  prev_mfinger_x + (0.145 + g_cube_d / 6 - prev_mfinger_x) * 10 * (time - stime);
		if (time - stime < step1 - 0.05)
		{
			target_cog_x = cog_x - g_cube_d / 3 + (target_cog_x - (cog_x - g_cube_d / 3)) * sin(PI * 10 * (time - stime));

			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, ys, z, LEFT_FINGER);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, -ys, z, RIGHT_FINGER);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, x, ys + 0.01, z + 0.030 /*0.135*/, MIDDLE_FINGER);
		}
		else
		{
			//for (jnt = 0; jnt < HAND_JNT; jnt++)
			//	ref_jnt_ang[jnt] = prev_jnt_ang[jnt];
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
		//theta = PI / 2.0 + THETA_S + sin(PI / 2 * 10.0 * (time - stime - step1)) * 90 * (PI / 180); //* PI / 2.0;//20.0 * (time - stime - step1) * PI / 2.0;
		theta = PI / 2.0 + THETA_S + (PI / 2) * 10.0 * (time - stime - step1); //20.0 * (time - stime - step1) * PI / 2.0;
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
		ref_jnt_ang[HAND_M4] = PI / 2; //
	}
	//step3 指先を元に戻す
	else if (time - stime < step3)
	{
		target_y = g_cube_d / 2 + r + 0.01;
		theta = PI / 2.0 + THETA_S + PI / 2.0;
		x = X + FO * cos(theta) - 0.002;
		y = Y + FO * sin(theta) + (target_y - (Y + FO * sin(theta))) * 20.0 * (time - stime - step25);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, x, y, z + 0.030 /*0.135*/, MIDDLE_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, ys, z, LEFT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, -ys, z, RIGHT_FINGER);
		//指先がキューブに接触してしまうので90曲げる
		ref_jnt_ang[HAND_M4] = PI / 2; //
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
		target_y = g_cube_d / 2 + r + 0.01;
		theta = PI / 2.0 + THETA_S + PI / 2.0;
		x = X + FO * cos(theta) - 0.002;
		//double x = X + FO * cos(theta) - 0.002 + 0.01 * 10 * (time - stime - step3);
		if (time - stime < step4 - 0.05)
		{
			target_x = x + (x_f_end - x) * sin(PI * 10 * (time - stime - step3));
			target_cog_x = target_cog_x + (x_f_end - target_cog_x) * 10 * (time - stime - step3);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_x, target_y, z + 0.030 /*0.135*/, MIDDLE_FINGER);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, ys, z, LEFT_FINGER);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, target_cog_x, -ys, z, RIGHT_FINGER);
		}
		else
		{
			//for (jnt = 0; jnt < HAND_JNT; jnt++)
			//	ref_jnt_ang[jnt] = prev_jnt_ang[jnt];
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
	double offset = 0.003;			  //把持力に関係している　大きくすると強く把持してキューブが回転しないかもしれない。小さくてもキューブが把持できない。
	double xs = cog_x - g_cube_d / 3; //キューブの1列目と2列目の中間を持つ
	double ys = g_cube_d / 2 + r - offset;
	double zs = g_Lw0; //hnad2013は0.105[m]

	double xt = 0.110; //持ち上げたときの把持位置
	double xe = 0.1265;
	double ze = 0;
	double base_h = table_h;									 //テーブルの高さhand2013は0.28
	double base_r = Lb;											 //土台からロボット座標の原点までの高さ hand2013は0.195
	double z = g_Lw0;											 //hand2013は0.105[m]
	double z_top = base_h - base_r + (5 * g_cube_d) / 6 + 0.004; //キューブの上層と中層の間のZ座標　0.005は少し持ち上げたほうが回転ミスが減るから
	double z_bottom = base_h - base_r + g_cube_d / 6;			 //一番下の把持位置 キューブの底面ブロックの真ん中のZ座標

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
		//指をキューブから離したあと把持位置まで指先を移動する
		if (time - stime < step1 - 0.05)
		{

			//xs = xe + 0.01;		  //xs = xe + (xs - xe) * 10 * (time - stime);
			xs = 0.125;
			ys = ys + 5 * offset; // + 10 * 2 * offset * (time - stime);
			z = zs - (zs - z_bottom) * 10 * (time - stime);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, -ys, z, RIGHT_FINGER);
			invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, ys, z, LEFT_FINGER);
		}
		else
		{
			xs = xe + (xs - xe) * 10 * (time - stime - 0.05);
			ys = ys + 5 * offset - 10 * 2 * 5 * offset * (time - stime - 0.05);
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
		xs = xs + (xt - xs) * 5 * (time - stime - step15);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, -ys, z, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs, ys, z, LEFT_FINGER);
	}
	//step3　指先を元の位置に戻す
	else if (time - stime < step3)
	{
		//z = z_top - (z_top - z) * sin(5 * PI * (time - stime - step2));
		z = z_top - (z_top - z) * 10 * (time - stime - step2);
		//ys = ys + 2 * offset;
		xt = xt + (xs - xt) * 10 * (time - stime - step2);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xt, -ys - 0.00, z, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xt, ys + 0.00, z, LEFT_FINGER);
		if (time - stime > step3 - 0.001)
		{
			cog_write = 1;
		}
	}
	ref_jnt_ang[HAND_M3] = prev_jnt_ang[HAND_M3];
	ref_jnt_ang[HAND_M4] = prev_jnt_ang[HAND_M4];
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
	double offset = 0.005; //把持力に関係している　大きくすると強く把持してキューブが回転しないかもしれない。小さくてもキューブが把持できない。
	double xs = 0.125;	 //cog_x - g_cube_d / 6;//キューブ手前列の真ん中を持つ

	double ys = g_cube_d / 2 + r - offset;
	double z = g_Lw0; //0.105;
	double step1 = 0.025;
	double step2 = 0.075;
	double step3 = 0.1;

	double theta_s = 0.0;
	double theta_e = 0.0;
	double PO_R = 0.0; //弾く方　段々半径が小さくなっていく
	double PO_L = 0.0;
	double FO_R = 0.0;
	double FO_L = 0.0;
	double theta_f = 0.0;
	/*
	double theta_s = 0.58;
	double theta_e = PI / 6;
	double FP = 0.0085; //人差し指半径　人差し指指先関節リンク長は0.0525
	double QO = g_cube_d / 2;
	double PO_R = QO / cos(theta_s); //弾く方　段々半径が小さくなっていく
	double PO_L = QO / cos(theta_s);
	double FO_R = sqrt(FP * FP + PO_R * PO_R - 2 * FP * PO_R * cos(PI - theta_s));
	double FO_L = sqrt(FP * FP + PO_L * PO_L - 2 * FP * PO_L * cos(PI - theta_s));
	double theta_f = theta_s - acos((FO_R * FO_R + PO_R * PO_R - FP * FP) / (2 * FO_R * PO_R));
	*/

	int jnt;
	//ready
	if (time - stime < step1)
	{

		theta_s = 0.588;
		theta_e = PI / 9;
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
		//for (jnt = 0; jnt < HAND_JNT; jnt++)
		for (jnt = 0; jnt < HAND_JNT; jnt++)
		{
			ref_jnt_ang[jnt] = prev_jnt_ang[jnt];
			tmp_jnt_ang[jnt] = prev_jnt_ang[jnt];
		}
		//invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, cog_x - g_cube_d / 3, -ys, z, RIGHT_FINGER);
		//invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, cog_x - g_cube_d / 3, ys, z, LEFT_FINGER);

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
			//ref_jnt_ang[HAND_M6] += -PI / 5 * (1.0 - cos(2.0 * PI * (time - stime - step1) * 10));
			//ref_jnt_ang[HAND_M2] += PI / 5 * (1.0 - cos(2.0 * PI * (time - stime - step1) * 10));
			ref_jnt_ang[HAND_M6] = tmp_jnt_ang[HAND_M6] - PI / 7 * (1.0 - cos((PI / (step2 - step1)) / 2 * (time - stime - step1)));
			ref_jnt_ang[HAND_M2] = tmp_jnt_ang[HAND_M2] + PI / 7 * (1.0 - cos((PI / (step2 - step1)) / 2 * (time - stime - step1)));
			//ref_jnt_ang[HAND_M1] = PI/60;
		}
		else if (clock_wise == COUNTER_CLOCKWISE)
		{
			ref_jnt_ang[HAND_M6] = tmp_jnt_ang[HAND_M6] + PI / 7 * (1.0 - cos((PI / (step2 - step1)) / 2 * (time - stime - step1)));
			ref_jnt_ang[HAND_M2] = tmp_jnt_ang[HAND_M2] - PI / 7 * (1.0 - cos((PI / (step2 - step1)) / 2 * (time - stime - step1)));
			//ref_jnt_ang[HAND_M5] = PI/60;
		}
		//指先でルービックキューブを回転させた後指をキューブから離す必要があるが、離すのが遅いとルービックキューブと指先が接触してキューブの速度が減速してしまい失敗するので指先で回し切る前に指根元関節を離す
		if (time - stime > step2 - 0.01)
		{
			ref_jnt_ang[HAND_M1] = -PI / 6;
			ref_jnt_ang[HAND_M5] = -PI / 6;
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
			ref_jnt_ang[HAND_M1] = -PI / 6;
			ref_jnt_ang[HAND_M5] = -PI / 6;
		}
		else if (clock_wise == COUNTER_CLOCKWISE)
		{
			if (time - stime > step2 + 0.01)
			{
				ref_jnt_ang[HAND_M1] = -PI / 6;
			}
			//ref_jnt_ang[HAND_M1] = -PI / 6;
			ref_jnt_ang[HAND_M5] = -PI / 6;
		}
	}
	//Catch
	else if (angle > 70.0 || angle < 20.0)
	{
		tan_vec_x_r = 0; // * tan_vec_x_r;
		tan_vec_y_r = 0; // * tan_vec_y_r;
		tan_vec_x_l = 0; // * tan_vec_x_l;
		tan_vec_y_l = 0; // * tan_vec_y_l;
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs + g_cube_d / 6, -ys, z, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, xs + g_cube_d / 6, ys, z, LEFT_FINGER);
	}
	else
	{
		if (clock_wise == CLOCKWISE)
		{
			ref_jnt_ang[HAND_M1] = -PI / 6;
			ref_jnt_ang[HAND_M2] = PI / 4; //prev_jnt_ang[HAND02_M2];
			ref_jnt_ang[HAND_M5] = -PI / 6;
			ref_jnt_ang[HAND_M6] = PI / 4; //prev_jnt_ang[HAND02_M6];
		}
		else if (clock_wise == COUNTER_CLOCKWISE)
		{
			ref_jnt_ang[HAND_M1] = -PI / 6;
			ref_jnt_ang[HAND_M2] = PI / 4; //prev_jnt_ang[HAND02_M2];
			ref_jnt_ang[HAND_M5] = -PI / 6;
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

//指先の転がりも考慮したY動作
void Yturn_Rolling(HANDJnt ref_jnt_ang, HANDJnt prepare_jnt_ang1, double TRAJ_RATE3, double time, double stime, double angle, double *camera, int clock_wise)
{
	double d;						//ロボット座標系原点からみた指根関節の座標
	double a_1;						//指根元関節角
	double a_2;						//指先関節角
	double r = 0.045 - 0.0365;		//指先半径
	double cube_angle;				//弧度法で表したキューブの角度
	double f;						//指先の接触点の角度
	double L_1 = 0.062;				//指先根元リンク
	double f2;						// = a_1 + a_2 + (90 - angle) * PI / 180 - PI / 2;
	double L_2 = 0.0365;			//指先リンク
	double x_f;						//指先のx座標
	double y_f;						//指先のy座標
	double omega = PI/2;				//目標とするルービックキューブの角速度
	double O_x = camera[0] - 0.045; //ロボット座標系から見たルービックキューブの重心位置のx座標
	double O_y = camera[1];			//ロボット座標系から見たルービックキューブの重心位置のy座標
	double u_1;						//接触点座標系からみたルービックキューブの重心の位置
	double u_3 = g_cube_d / 2;		//接触点座標系からみたルービックキューブの重心の位置
	double tmp;
	double tmp2;
	double A[3][3]; //逆行列
	double S12f;
	double C12f;
	double S12;
	double C12;
	double da_1; //関節角の速度
	double da_2; //関節角の速度
	double da_5; //関節角の速度
	double da_6; //関節角の速度
	if (angle > 45.0)
	{
		cube_angle = (90.0 - angle) * PI / 180.0;
	}
	else
	{
		cube_angle = -angle * PI / 180.0;
	}

	//キューブの回転方向によって軌道を変える
	//左指
	d = 0.06;
	
	a_1 = hand.var.jnt_ang[HAND_M1]; //指根元関節角
	a_2 = hand.var.jnt_ang[HAND_M2];
	x_f = L_2 * cos(a_1 + a_2) + L_1 * cos(a_1);	  //指先のx座標
	y_f = -L_2 * sin(a_1 + a_2) - L_1 * sin(a_1) + d; //指先のy座標
	f = a_1 + a_2 + cube_angle - PI / 2;
	//u_1 = (tan(cube_angle) * x_f - y_f - tan(cube_angle) * O_x + O_y) / sqrt(tan(cube_angle) * tan(cube_angle) + 1);
	u_1 = (tan(cube_angle) * (-y_f) - x_f + O_x + tan(cube_angle) * O_y) / sqrt(tan(cube_angle) * tan(cube_angle) + 1);
	tmp = sin(a_2 - f) * u_3 - cos(a_2 - f) * u_1 - L_2 * sin(a_2);
	tmp2 = sin(a_2 - f) * u_3 + cos(a_2 - f) * u_1 - L_2 * sin(a_2);
	S12f = sin(a_1 + a_2 - f);
	C12f = cos(a_1 + a_2 - f);
	S12 = sin(a_1 + a_2);
	C12 = cos(a_1 + a_2);
	if (omega < 0.0)
	{
		u_1 = -u_1;
	}

	if (omega > 0.0)
	{
		A[0][0] = (C12f * u_3 + S12f * u_1 - L_2 * C12) / (L_1 * tmp);
		A[0][1] = -(S12f * u_3 - C12f * u_1 - L_2 * S12) / (L_1 * tmp);
		A[0][2] = -(d * C12f * u_3 + L_1 * sin(a_2 - f) * u_3 + d * S12f * u_1 - L_1 * cos(a_2 - f) * u_1 - L_2 * d * C12 - L_1 * L_2 * sin(a_2)) / (L_1 * tmp);

		A[1][0] = -(C12f * u_3 + S12f * u_1 - L_2 * C12 - L_1 * cos(a_1)) / (L_1 * tmp);
		A[1][1] = (S12f * u_3 - C12f * u_1 - L_2 * S12 - L_1 * sin(a_1)) / (L_1 * tmp);
		A[1][2] = (d * C12f * u_3 + S12f * u_1 - L_2 * C12 - L_1 * cos(a_1)) / (L_1 * tmp);
	}
	else
	{
		A[0][0] = (C12f * u_3 - S12f * u_1 - L_2 * C12) / (L_1 * tmp2);
		A[0][1] = -(S12f * u_3 + C12f * u_1 - L_2 * S12) / (L_1 * tmp2);
		A[0][2] = -(d * C12f * u_3 + L_1 * sin(a_2 - f) * u_3 - d * S12f * u_1 + L_1 * cos(a_2 - f) * u_1 - L_2 * d * C12 - L_1 * L_2 * sin(a_2)) / (L_1 * tmp2);

		A[1][0] = -(C12f * u_3 - S12f * u_1 - L_2 * C12 - L_1 * cos(a_1)) / (L_1 * tmp2);
		A[1][1] = (S12f * u_3 + C12f * u_1 - L_2 * S12 - L_1 * sin(a_1)) / (L_1 * tmp2);
		A[1][2] = (d * C12f * u_3 - S12f * u_1 - L_2 * C12 - L_1 * cos(a_1)) / (L_1 * tmp2);
	}
	da_1 = A[0][0] * (O_y * omega) + A[0][1] * (-O_x * omega) + A[0][2] * omega;
	da_2 = A[1][0] * (O_y * omega) + A[1][1] * (-O_x * omega) + A[1][2] * omega;
	ref_jnt_ang[HAND_M1] = a_1 + da_1 / 1000;
	ref_jnt_ang[HAND_M2] = a_2 + da_2 / 1000;
	//ref_jnt_ang[HAND_M9] = u_1;
	//右指
	/*
	d = -d;
	a_1 = -hand.var.jnt_ang[HAND_M5]; //指根元関節角
	a_2 = -hand.var.jnt_ang[HAND_M6];
	x_f = L_2 * cos(a_1 + a_2) + L_1 * cos(a_1);	  //指先のx座標
	y_f = -L_2 * sin(a_1 + a_2) - L_1 * sin(a_1) + d; //指先のy座標
	f = a_1 + a_2 + cube_angle + PI / 2;
	//u_1 = -(tan(cube_angle) * x_f - y_f - tan(cube_angle) * O_x + O_y) / sqrt(tan(cube_angle) * tan(cube_angle) + 1);
	u_1 = -(tan(cube_angle) * (-y_f) - x_f + O_x + tan(cube_angle) * O_y) / sqrt(tan(cube_angle) * tan(cube_angle) + 1);
	tmp = sin(a_2 - f) * u_3 - cos(a_2 - f) * u_1 - L_2 * sin(a_2);
	tmp2 = sin(a_2 - f) * u_3 + cos(a_2 - f) * u_1 - L_2 * sin(a_2);
	S12f = sin(a_1 + a_2 - f);
	C12f = cos(a_1 + a_2 - f);
	S12 = sin(a_1 + a_2);
	C12 = cos(a_1 + a_2);
	if (omega < 0)
	{
		u_1 = -u_1;
	}
	if (omega > 0)
	{
		A[0][0] = (C12f * u_3 + S12f * u_1 - L_2 * C12) / (L_1 * tmp);
		A[0][1] = -(S12f * u_3 - C12f * u_1 - L_2 * S12) / (L_1 * tmp);
		A[0][2] = -(d * C12f * u_3 + L_1 * sin(a_2 - f) * u_3 + d * S12f * u_1 - L_1 * cos(a_2 - f) * u_1 - L_2 * d * C12 - L_1 * L_2 * sin(a_2)) / (L_1 * tmp);

		A[1][0] = -(C12f * u_3 + S12f * u_1 - L_2 * C12 - L_1 * cos(a_1)) / (L_1 * tmp);
		A[1][1] = (S12f * u_3 - C12f * u_1 - L_2 * S12 - L_1 * sin(a_1)) / (L_1 * tmp);
		A[1][2] = (d * C12f * u_3 + S12f * u_1 - L_2 * C12 - L_1 * cos(a_1)) / (L_1 * tmp);
	}
	else
	{
		A[0][0] = (C12f * u_3 - S12f * u_1 - L_2 * C12) / (L_1 * tmp2);
		A[0][1] = -(S12f * u_3 + C12f * u_1 - L_2 * S12) / (L_1 * tmp2);
		A[0][2] = -(d * C12f * u_3 + L_1 * sin(a_2 - f) * u_3 - d * S12f * u_1 + L_1 * cos(a_2 - f) * u_1 - L_2 * d * C12 - L_1 * L_2 * sin(a_2)) / (L_1 * tmp2);

		A[1][0] = -(C12f * u_3 - S12f * u_1 - L_2 * C12 - L_1 * cos(a_1)) / (L_1 * tmp2);
		A[1][1] = (S12f * u_3 + C12f * u_1 - L_2 * S12 - L_1 * sin(a_1)) / (L_1 * tmp2);
		A[1][2] = (d * C12f * u_3 - S12f * u_1 - L_2 * C12 - L_1 * cos(a_1)) / (L_1 * tmp2);
	}
	//A[0][0] = (C12f * u_3 + S12f * u_1 - L_2 * C12) / (L_1 * tmp);
	//A[0][1] = (C12f * u_3 + S12f * u_1 - L_2 * C12) / (L_1 * tmp);
	//A[0][2] = (C12f * u_3 + S12f * u_1 - L_2 * C12) / (L_1 * tmp);
	da_5 = A[0][0] * (O_y * omega) + A[0][1] * (-O_x * omega) + A[0][2] * omega;
	da_6 = A[1][0] * (O_y * omega) + A[1][1] * (-O_x * omega) + A[1][2] * omega;
	ref_jnt_ang[HAND_M5] = -a_1 - da_5 / 1000;
	ref_jnt_ang[HAND_M6] = -a_2 - da_6 / 1000;
	//ref_jnt_ang[HAND_M9] = u_1;
	*/
}

//回転記号の配列を受け取り順番に実行する
void execFromString(HANDJnt ref_jnt_ang, HANDJnt prepare_jnt_ang, double TRAJ_RATE3, double time, double *camData, int *operation)
{
	//const int opNum = sizeof operation / sizeof operation[0];
	//int opNum = 4;
#define opNum (30)
	//operation[n]がop_time[n]の時間に終了する
	//op_time[n]にoperation[n]の操作が終了する時間を入れる
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
	double offset = 0.0; //把持力に関係している　大きくすると強く把持してキューブが回転しないかもしれない。小さくてもキューブが把持できない。
	double xs = 0.0;	 //キューブ手前列の真ん中を持つ
	double ys = 0.0;	 //- offset;// - 0.004; //+ 2 * offset;
	double zs = 0.0;
	double x_m = 0.0;
	double y_m = 0.0;
	double z_m = 0.0;
	double theta_m_s = PI / 6;
	double theta_m = 0.0;
	double interval = 0.0;

	int i = 0;
	//data[0] = camData[0];
	//for (jnt = 0; jnt < HAND_JNT; jnt++)
	//	ref_jnt_ang[jnt] = prepare_jnt_ang[jnt];
	for (jnt = 0; jnt < HAND_JNT; jnt++)
		ref_jnt_ang[jnt] = prepare_jnt_ang[jnt];

	//前回(0.001秒前)の値を最初に入れておいてその後変更する
	//逆運動学に解がなかった場合に角度が0になってしまうことを防止するため
	//for (jnt = 0; jnt < HAND_JNT; jnt++)
	//	ref_jnt_ang[jnt] = prev_jnt_ang[jnt];
	for (jnt = 0; jnt < HAND_JNT; jnt++)
		ref_jnt_ang[jnt] = prev_jnt_ang[jnt];
	//デバック用
	/*
	double x = camera[1];
	double y = g_cube_d / 2 + r + 0.005;
	double z = 0.105;
	invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, x, -y, z, 1);
	invKine(ref_jnt_ang, prepare_jnt_ang1, time, 0.0, 0.0, x, y, z, 0);
	*/

	//初期位置へ移動
	if (time < 2.000)
	{
		if (time < 0.5)
		{
			cog_x = camData[0];
			cog_y = camData[1];
		}
		offset = 0.005;					//把持力に関係している　大きくすると強く把持してキューブが回転しないかもしれない。小さくてもキューブが把持できない。
		xs = cog_x - g_cube_d / 3;		//キューブ手前列の真ん中を持つ 前のハンドでは手前列の真ん中より更に手前を把持している。　リンクの長さが間違っていた可能性がある
		ys = g_cube_d / 2 + r - offset; //- offset;// - 0.004; //+ 2 * offset;
		zs = g_Lw0;
		prev_mfinger_x = cog_x - g_cube_d / 3;
		prev_mfinger_y = g_cube_d / 2 + r + 0.004;

		theta_m = PI / 2.0 + theta_m_s;
		x_m = cog_x + FO * cos(theta_m);
		y_m = cog_y + FO * sin(theta_m);

		invKine(ref_jnt_ang, prepare_jnt_ang, time, 0.0, 0.0, xs, -ys, zs, RIGHT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang, time, 0.0, 0.0, xs, ys, zs, LEFT_FINGER);
		invKine(ref_jnt_ang, prepare_jnt_ang, time, 0.0, 0.0, xs, prev_mfinger_y, zs + 0.030 /*0.135*/, MIDDLE_FINGER);

		//指先は根元関節より早く曲げる
		//物体に接触するまえに曲げてしまうことで物体が動かないようにする
		if (time < 1.0)
		{
			ref_jnt_ang[HAND_M1] = time * (-PI / 4);
			ref_jnt_ang[HAND_M3] = time * (-PI / 4);
			ref_jnt_ang[HAND_M5] = time * (-PI / 4);
			ref_jnt_ang[HAND_M2] *= time;
			ref_jnt_ang[HAND_M4] *= time;
			ref_jnt_ang[HAND_M6] *= time;
			ref_jnt_ang[HAND_M9] = 0;
		}
		else
		{
			ref_jnt_ang[HAND_M1] = -PI / 4 + (time - 1.0) * (ref_jnt_ang[HAND_M1] + PI / 4);
			ref_jnt_ang[HAND_M3] = -PI / 4 + (time - 1.0) * (ref_jnt_ang[HAND_M3] + PI / 4);
			ref_jnt_ang[HAND_M5] = -PI / 4 + (time - 1.0) * (ref_jnt_ang[HAND_M5] + PI / 4);
			ref_jnt_ang[HAND_M9] *= ref_jnt_ang[HAND_M9] * (time - 1);
		}
	}
	//	行いたい操作を順番に記号で入れる配列
	//int operation[] = {U_TURN_P, Y_ROTATION, X_ROTATION, U_TURN_P};
	//int operation[] = {U_TURN_P, Y_ROTATION, X_ROTATION, U_TURN_P, Y_ROTATION, X_ROTATION, U_TURN_P, Y_ROTATION, X_ROTATION, U_TURN_P, Y_ROTATION, X_ROTATION};
	//execFromString(ref_jnt_ang, prepare_jnt_ang, TRAJ_RATE3, time, camData, operation);

	//転がり接触を利用したルービックキューブの傾き制御テスト
	if (time >= 2.000 && time < 3.000)
	{
		Yturn_Rolling(ref_jnt_ang, prepare_jnt_ang, TRAJ_RATE3, time, 2.0, camData[2], camData, COUNTER_CLOCKWISE);
	}
	//関節角を保存
	//ref_jnt_ang[8] = hand.var.jnt_ang[0];
	for (jnt = 0; jnt < HAND_JNT; jnt++)
		prev_jnt_ang[jnt] = ref_jnt_ang[jnt];
	return 0;
#else
#define TRAJ_RAD PI / 8.0
#define TRAJ_RATE1 0.1
#define TRAJ_RATE2 1.0
#define TRAJ_RATE3 10.0

#define DEMO1_TIME1 5.0
#define DEMO1_TIME2 7.5
#define DEMO1_TIME3 9.0
	int jnt;
	double rate = 0.0;
	for (jnt = 0; jnt < HAND_JNT; jnt++)
		ref_jnt_ang[jnt] = prepare_jnt_ang[jnt];

	ref_jnt_ang[HAND_M2] = PI / 2;
	ref_jnt_ang[HAND_M4] = PI / 2;
	ref_jnt_ang[HAND_M6] = PI / 2;
	/*

	// ref_jnt_ang[HAND_M1] = -TRAJ_RAD + prepare_jnt_ang[HAND_M1];
	// ref_jnt_ang[HAND_M3] = -TRAJ_RAD + prepare_jnt_ang[HAND_M3];
	// ref_jnt_ang[HAND_M5] = -TRAJ_RAD + prepare_jnt_ang[HAND_M5];
	//ref_jnt_ang[HAND_M2] = TRAJ_RAD + prepare_jnt_ang[HAND_M2];
	//ref_jnt_ang[HAND_M4] = TRAJ_RAD + prepare_jnt_ang[HAND_M4];
	//ref_jnt_ang[HAND_M6] = TRAJ_RAD + prepare_jnt_ang[HAND_M6];
	if (time < DEMO1_TIME1)
	{
		ref_jnt_ang[HAND_M1] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M1];
		ref_jnt_ang[HAND_M3] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M3];
		ref_jnt_ang[HAND_M5] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M5];
		ref_jnt_ang[HAND_M2] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M2];
		ref_jnt_ang[HAND_M4] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M4];
		ref_jnt_ang[HAND_M6] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M6];
		//ref_jnt_ang[HAND_M7] = PI/3;

		//ref_jnt_ang[HAND_M7] = -TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE1)) + prepare_jnt_ang[HAND_M7];
		//ref_jnt_ang[HAND_M8] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE1)) + prepare_jnt_ang[HAND_M8];

		ref_jnt_ang[HAND_M9] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M9];
		//ref_jnt_ang[HAND_M10] = -TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE1)) + prepare_jnt_ang[HAND_M10];

		ref_jnt_ang[HAND_M11] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE1)) + prepare_jnt_ang[HAND_M11];
	}
	else if (time < DEMO1_TIME2)
	{
		rate = 2.0 * PI * DEMO1_TIME1 * TRAJ_RATE1;
	
		ref_jnt_ang[HAND_M1] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M1];
		ref_jnt_ang[HAND_M3] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M3];
		ref_jnt_ang[HAND_M5] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M5];
		ref_jnt_ang[HAND_M2] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M2];
		ref_jnt_ang[HAND_M4] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M4];
		ref_jnt_ang[HAND_M6] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M6];

		ref_jnt_ang[HAND_M9] = TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M9];
		//ref_jnt_ang[HAND_M10] = -TRAJ_RAD * (1.0 - cos(2.0 * PI * time * TRAJ_RATE2 - rate)) + prepare_jnt_ang[HAND_M10];
	}
	else if (time <DEMO1_TIME3) {
	rate = 2.0*PI*DEMO1_TIME2*TRAJ_RATE2;
	ref_jnt_ang[HAND_M1] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE3 - rate)) + prepare_jnt_ang[HAND_M1];
	ref_jnt_ang[HAND_M3] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE3 - rate)) + prepare_jnt_ang[HAND_M3];
	ref_jnt_ang[HAND_M5] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE3 - rate)) + prepare_jnt_ang[HAND_M5];
	ref_jnt_ang[HAND_M2] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE3 - rate)) + prepare_jnt_ang[HAND_M2];
	ref_jnt_ang[HAND_M4] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE3 - rate)) + prepare_jnt_ang[HAND_M4];
	ref_jnt_ang[HAND_M6] = TRAJ_RAD*(1.0 - cos(2.0*PI*time*TRAJ_RATE3 - rate)) + prepare_jnt_ang[HAND_M6];
	}
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
	}*/
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
	hand->base_homo = &hand_base_homo;
	hand->ctrl_coef = &hand_ctrl_coef;
	hand->init_jnt_ang = init_jnt_ang;
	hand->prepare_jnt_ang = prepare_jnt_ang;
	return 0;
}

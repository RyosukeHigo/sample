#ifndef WAM2_TRAJ_H_INCLUDED
#define WAM2_TRAJ_H_INCLUDED

int rangeSelector(const double z);	//捕球点のz座標から合成する軌道を選択

//スケール変換を考慮した関節角軌道生成
double waveGeneAxis1(const double motion_time, double scale, int prevMotion); //1軸
double waveGeneAxis3(const double motion_time, const double catch_jnt, const double z_pos, double scale); //3軸
double waveGeneAxis4(const double motion_time, const double catch_jnt, const double z_pos, double scale); //4軸

//初期関節角
#define PREPARE_WAM2_AXIS1 (1.1254) //WAM2の1軸の初期関節角
//const static WAMJnt	prepare_jnt_ang = {0.6283, PI/5, 0.0, PI/12, 0.0};
//const static WAMJnt	prepare_jnt_ang = {1.1254, PI/5, 0.0, PI/12, 0.0}; //2012.12.19 とりあえず1軸だけ定数で変える様にした．
//int positionFlag;	//ボールが通りすぎたかどうか　20090618 とりあえずwam2App.cへ戻す

// 捕球までの時間
#define BEFORE_CATCHING_TIME		(0.23)	//0.3690
#define BEFORE_CATCHING_TIME_FULL	(0.369) //1軸の最大捕球前時間
#define BEFORE_CATCHING_TIME_AX34	(0.169) //3,4軸の捕球前の時間
#define DELAY_TIME					(BEFORE_CATCHING_TIME - BEFORE_CATCHING_TIME_AX34) //3,4軸が1,2軸に対して遅れる時間（秒）
#define SHORTCUT_TIME				(BEFORE_CATCHING_TIME_FULL - BEFORE_CATCHING_TIME) //捕球前時間の最大時間に比べて何秒短いか
#define SWING_TIME					(1.0) //1軸の元の動作の動作時間（秒）
#define SWING_TIME_AX34				(0.7) //3,4軸の動作時間(秒)
#define MIN_X						(3.9)  //用意した関数の一番奥
#define MAX_X						(3.3)  //用意した関数の一番奥
#define SWING_OFFSET				(0)//(0.125)//(0.03)	// ボールの減速考慮のためのオフセット
#define AFTER_CATCHING_TIME		    (0.60) //捕球後最終位置に戻るまでの時間

#define MAX_Z	(0.70)	//捕球点の許容最大z座標
#define MIN_Z	(0.35)	//捕球点の許容最小z座標			

#define SCALE_BUFFER_COUNT (5)	//スケール変換後，捕球点で通常の速度で捕球するための余剰カウント数
#define MIN_SCALE_APPLY_COUNT (15+SCALE_BUFFER_COUNT)	//スケール変換を適用する最小カウント数

#define AFTER_LAUNCH_TIME (0.546) //発射から捕球までの平均的な時間

#endif /*WAM2_TRAJ_H_INCLUDED*/
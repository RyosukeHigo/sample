// Usage:
//
//    predict3D_init();                // 初期化
//    ...
//    predict3D_start(obj_pos, count); // 予測開始
//    loop{
//          if(obj_pos is observed) predict3D(obj_pos_predcit, count_bias, obj_pos, count, interval);
//          else                    predict3D_calc(obj_pos_predcit, count_bias, count, interval);
//    }
//   predict3D_exit();                 // 終了処理

#ifndef PREDICT_3D_H
#define PREDICT_3D_H
#include <Kine.h>

//////////////////////////////////////////////////
// 初期化
//////////////////////////////////////////////////
void predict3D_init(void);

//////////////////////////////////////////////////
// 予測開始
//////////////////////////////////////////////////
void predict3D_start(Vector3D obj_pos,  // 対象の位置
		     int      count);   // 開始時のループカウンタ  

//////////////////////////////////////////////////
// 対象軌道のパラメータの更新と軌道予測 
// 返値: obj_pos_predict
//     ループカウンタが (count_bias + count)の時の
//     対象軌道 obj_pos_predict を予測する．
//////////////////////////////////////////////////
void predict3D(Vector3D obj_pos_predict,  // 対象の予測位置
	       int      count_bias,       // 予測時点のカウンタ値 - 現カウンタ値
	       Vector3D obj_pos,          // 現時点での対象の位置
	       int      count,            // 現時点でのループカウンタ
	       double    interval);        // サイクルタイム（通常 0.001[s]）

//////////////////////////////////////////////////
// 対象軌道の予測 （軌道パラメータの更新を行わない）
// 返値: obj_pos_predict
//     ループカウンタが (count_bias + count)の時の
//     対象軌道 obj_pos_predict を予測する．
//////////////////////////////////////////////////
void predict3D_calc(Vector3D obj_pos_predict,   // 対象の予測位置
		    int      count_bias,        // 予測時点のカウンタ値 - 現カウンタ値
		    int      count,             // 現時点でのループカウンタ
		    double    interval);         // サイクルタイム（通常 0.001[s]）

//////////////////////////////////////////////////
// 対象軌道のパラメータの更新と軌道パラメータの取得
// 返値: a[6]
//////////////////////////////////////////////////
void predict_trajectory(Vector3D obj_pos,
		  double a[6],
		  int count,
		  double interval);

//////////////////////////////////////////////////
// 終了処理
//////////////////////////////////////////////////
void predict3D_exit(void);


//////////////////////////////////////////////////
// 軌道パラメータの取得
// 返値: a[6]
// 対象軌道obj_pos_predictは
//    obj_pos_predict[0] = a[3]*t*t + a[0]*t + obj_pos_init[0];
//    obj_pos_predict[1] = a[4]*t*t + a[1]*t + obj_pos_init[1];
//    obj_pos_predict[2] = a[5]*t*t + a[2]*t + obj_pos_init[2];
// で予測される．obj_pos_init は予測開始時点での対象の位置, tは時間．
//////////////////////////////////////////////////
void predict3D_get(double a[6]);

#endif

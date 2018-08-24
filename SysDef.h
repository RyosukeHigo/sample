#ifndef SYSDEF_H
#define SYSDEF_H

//////////////////////////////////////////////////
// システム使用設定
// 使用するときは 1 , 使用しないときは 0 を設定
// HAND1: 02ハンド
// HAND2: バレット用ハンド
// WAM1: バッティングアーム
// WAM2: スローイングアーム
//////////////////////////////////////////////////
#define SYSTEM_HAND1	0		// ０２ハンド
#define SYSTEM_HAND2	1		// バレット用ハンド
#define SYSTEM_WAM1     0		// バッティング用アーム
#define SYSTEM_WAM2     0		// スローイング用アーム
#define SYSTEM_XYZ  	1
#define SYSTEM_AVS      0
#define SYSTEM_FORCE	0
#define SYSTEM_HAND_WAM_CONNECT		0		// ハンドをアームに搭載
#define SYSTEM_HAND		(SYSTEM_HAND1 || SYSTEM_HAND2)		// ハンドをどちらか使う場合
#define SYSTEM_WAM      (SYSTEM_WAM1 || SYSTEM_WAM2)		// アームをどちらか使う場合
#define SYSTEM_WAM_NUM  (SYSTEM_WAM1 + SYSTEM_WAM2)		// アームを使う台数

// ENC
#define NORMALIZE_ENC	    pow(2.0,31) //ds3002
#define NORMALIZE_ENC_STAGE pow(2.0,23) //ds3001
// DA (scan table number は4つまで)
#define TBL_HAND 1
#define TBL_XYZ  2
#define TBL_WAM  3
#define TBL_SERVO  3
#define TBL_AVS  4
#define DA_LIMIT_HAND	1.0
#define DA_LIMIT_WAM	0.45		// マニュアル上は0.42(4.2V)まで
#define DA_LIMIT_AVS	1.0
#define DA_LIMIT_XYZ	0.9

#endif

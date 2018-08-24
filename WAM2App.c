#include <stdio.h>
#include <math.h>
#include <common_definition.h>
#include <WAM.h>
#include <WAMFunc.h>
#include "App.h"
#include "SysDef.h"
#include "Data.h"

// \‘¢‘ÌéŒ¾
WAM		wam2;


// ƒOƒ[ƒoƒ‹À•W‚É‚¨‚¯‚éƒA[ƒ€ˆÊ’u
static HomoMat wam_base_homo ={{1.0, 0.0, 0.0},
		                        {0.0, 1.0, 0.0},
		                        {0.0, 0.0, 1.0},
								{0.0, 0.675, 0.0}};
// §ŒäƒQƒCƒ“
static WAMCtrlCoef wam_ctrl_coef = {{8.0, 8.0, 8.0, 8.0, 1000.0},				// Kp
									{0.0, 0.0, 0.0, 0.0, 0.0},					// Ti
									{120.0, 120.0, 80.0, 80.0, 3000.0},			//Td
									{0.0, 0.0, 0.0, 0.0, 0.0},					// Cf
									100};										// Kg
// ‰Šúp¨
//const static WAMJnt	prepare_jnt_ang = {0.0, WAM_M2_INIT_JNT_ANG, 0.0, WAM_M4_INIT_JNT_ANG, 0.0};
const static WAMJnt	prepare_jnt_ang = {5*PI/6, 1.2, -5*PI/12, 0.8, 0.0};//PI/3,-PI/2-PI/12, PI/3.0, 0.0};


int battingModelJnt2armJnt(double *qa, double *dqa, double *ddqa,
					double *qt, double *dqt, double *ddqt);
int armJnt2battingModelJnt(double *qt, double *dqt, double *ddqt,
					double *qa, double *dqa, double *ddqa);

//////////////////////////////////////////////////////
// I—¹‘¬“xE‰Á‘¬“x‚Í0
double wamTra5th(const double time,					// 
				 const double start_time,			// 
				 const double all_time,				// Á´Æ°ºû×ş´Ö
		     const double start_jnt,			// ½é´EØÀá³Ñ
		     const double start_vel,				// ½ªÎ»´ØÀá³Ñ
		     const double start_acc,				// ½ªÎ»´ØÀá³Ñ
		     const double end_jnt)				// ½ªÎ»´ØÀá³Ñ
{
  double  motion_time;
  double  rate, t0;
  double  coef[6];
  motion_time = time - start_time;
	t0 = all_time;
	coef[0]=start_jnt;
	coef[1]=start_vel*t0;
	coef[2]=0.5*start_acc;
	coef[3]=10*(end_jnt-coef[0])-6*coef[1]-1.5*start_acc;
	coef[4]=-15*(end_jnt-coef[0])+8*coef[1]+1.5*start_acc;
	coef[5]=6*(end_jnt-coef[0])-3*coef[1]-0.5*start_acc;
	rate = motion_time / all_time;
	return coef[0]+coef[1]*rate+coef[2]*rate*rate+coef[3]*rate*rate*rate
	    +coef[4]*rate*rate*rate*rate+coef[5]*rate*rate*rate*rate*rate;
}


//////////////////////////////////////////////////////
double waveGeneSin(const double motion_time,					// ”g“®ŠÖ”ŠJn‚©‚ç‚ÌŠÔ
					 const double all_time,				// ‘S‘Ì‚ÌŠÔ
		     const double start_jnt,			// ‰ŠúŠÖßŠp
		     const double end_jnt)				// I’[ŠÖßŠp
{
  return  start_jnt + (end_jnt-start_jnt)*(1-cos(PI*motion_time/all_time))/2;
}

//////////////////////////////////////////////////////
double waveGeneExp(const double time,					// ”g“®ŠÖ”ŠJn‚©‚ç‚ÌŠÔ
					 const double t0,				// ‘S‘Ì‚ÌŠÔ/4
		     const double qs,			// ‰ŠúŠÖßŠp
		     const double qe)				// ”¼üŠúŒã‚ÌŠÖßŠp
{
	double alpha, omega, q0;
	alpha = 0.1;
	q0 = qs - alpha;
	omega = log(1+(qe-qs)/(2*alpha))/t0;
	if(time < t0)	return alpha*exp(omega*time)+q0;
	else if(time < 2*t0)	return -alpha*exp(-omega*(time-2*t0))+2*alpha*exp(omega*t0)+q0;
	else if(time < 3*t0)	return -alpha*exp(omega*(time-2*t0))+2*alpha*exp(omega*t0)+q0;
	else	return alpha*exp(-omega*(time-4*t0))+q0;
}

//////////////////////////////////////////////////////
// ƒXƒ[ƒCƒ“ƒO
//////////////////////////////////////////////////////
int wam2TrajApp(WAMJnt ref_jnt_ang, DATA *data, double time)
{
// 2²ŒÅ’è
#define WAM_M2_JNT_ANG		(PI/5)	//(PI/5)

#if 1
/////////////////////////////////////////
// ÀŒ±—p
/////////////////////////////////////////
// ‘±ŠÔ
#define TAKEBACK_TIME_M1	(3.0)
#define SWING_TIME_M1		(1.0)
// ŠJn‚©‚ç‚ÌŠÔ
#define TAKEBACK_START_M1	(4.0)
#define TAKEBACK_END_M1		(TAKEBACK_START_M1+TAKEBACK_TIME_M1)
#define SWING_START_M1		TAKEBACK_END_M1
#define SWING_END_M1		(SWING_START_M1+SWING_TIME_M1)

// ‘±ŠÔ
#define TAKEBACK_TIME_M3	(3.0)
#define SWING_TIME_M3		(0.7)	//(1.0)
// ŠJn‚©‚ç‚ÌŠÔ
#define TAKEBACK_START_M3	(4.0)
#define TAKEBACK_END_M3		(TAKEBACK_START_M3+TAKEBACK_TIME_M3)
#define SWING_START_M3		(TAKEBACK_END_M3 + 0.2)
#define SWING_END_M3		(SWING_START_M3+SWING_TIME_M3)

// ‘±ŠÔ
#define TAKEBACK_TIME_M4	(3.0)
#define SWING_TIME_M4		(0.7)
// ŠJn‚©‚ç‚ÌŠÔ
#define TAKEBACK_START_M4	(4.0)
#define TAKEBACK_END_M4		(TAKEBACK_START_M4+TAKEBACK_TIME_M4)
//#define SWING_START_M4		(TAKEBACK_END_M4 + 0.3)
#define SWING_START_M4		(TAKEBACK_END_M4 + 0.2)
#define SWING_END_M4		(SWING_START_M4+SWING_TIME_M4)

/////////////////////////////////////////
// ƒfƒoƒbƒO—p
/////////////////////////////////////////
#else
// ‘±ŠÔ
#define TAKEBACK_TIME_M1	(3.0)
#define SWING_TIME_M1		(0.9)
// ŠJn‚©‚ç‚ÌŠÔ
#define TAKEBACK_START_M1	(4.0)
#define TAKEBACK_END_M1		(TAKEBACK_START_M1+TAKEBACK_TIME_M1)
#define SWING_START_M1		TAKEBACK_END_M1
#define SWING_END_M1		(SWING_START_M1+SWING_TIME_M1)

// ‘±ŠÔ
#define TAKEBACK_TIME_M3	(3.0)
#define SWING_TIME_M3		(0.55)
// ŠJn‚©‚ç‚ÌŠÔ
#define TAKEBACK_START_M3	(4.0)
#define TAKEBACK_END_M3		(TAKEBACK_START_M3+TAKEBACK_TIME_M3)
#define SWING_START_M3		(TAKEBACK_END_M3 + 0.3)
#define SWING_END_M3		(SWING_START_M3+SWING_TIME_M3)

// ‘±ŠÔ
#define TAKEBACK_TIME_M4	(3.0)
#define SWING_TIME_M4		(0.55)
// ŠJn‚©‚ç‚ÌŠÔ
#define TAKEBACK_START_M4	(4.0)
#define TAKEBACK_END_M4		(TAKEBACK_START_M4+TAKEBACK_TIME_M4)
#define SWING_START_M4		(TAKEBACK_END_M4 + 0.3)
#define SWING_END_M4		(SWING_START_M4+SWING_TIME_M4)

#endif

	int	jnt;
	//WAMJnt	takeback_jnt_ang = {-PI/2, WAM_M2_JNT_ANG, -2*PI/3, PI/12, 0};
	//WAMJnt	swing_end_jnt_ang = {PI/2, WAM_M2_JNT_ANG, 0, PI/12, 0};
	WAMJnt	takeback_jnt_ang = {-PI/2, WAM_M2_JNT_ANG, 0, PI/2-WAM_M2_JNT_ANG, 0};
	WAMJnt	swing_end_jnt_ang = {PI/2, WAM_M2_JNT_ANG, 0, PI/2-WAM_M2_JNT_ANG, 0};
	WAMJnt	qt, dqt, ddqt;
	WAMJnt	prepare_qt;
	WAMJnt	dammy;
	WAMJnt _prepare_jnt_ang;

	// ‰Šúp¨‚ğƒ‚ƒfƒ‹ŠÖß‚É•ÏŠ·
	for(jnt=0;jnt<WAM_JNT;jnt++){
		_prepare_jnt_ang[jnt] = prepare_jnt_ang[jnt];
	}
	armJnt2battingModelJnt(prepare_qt, dammy, dammy, _prepare_jnt_ang, dammy, dammy);

	// sinŒ^
	if(time < TAKEBACK_START_M1)	qt[WAM_M1] = prepare_qt[WAM_M1];
	else if(time < TAKEBACK_END_M1)	qt[WAM_M1] = waveGeneSin(time-TAKEBACK_START_M1, TAKEBACK_TIME_M1, prepare_qt[WAM_M1], takeback_jnt_ang[WAM_M1]);
	else if(time < SWING_END_M1)	qt[WAM_M1] = waveGeneSin(time-SWING_START_M1, SWING_TIME_M1, takeback_jnt_ang[WAM_M1], swing_end_jnt_ang[WAM_M1]);
	else	qt[WAM_M1] = swing_end_jnt_ang[WAM_M1];

	// ŒÅ’è
	//qt[WAM_M2] = WAM_M2_JNT_ANG;

	// sinŒ^
	if(time < TAKEBACK_START_M3)	qt[WAM_M3] = prepare_qt[WAM_M3];
	else if(time < TAKEBACK_END_M3)	qt[WAM_M3] = waveGeneSin(time-TAKEBACK_START_M3, TAKEBACK_TIME_M3, prepare_qt[WAM_M3], takeback_jnt_ang[WAM_M3]);
	else if(time < SWING_START_M3)	qt[WAM_M3] = takeback_jnt_ang[WAM_M3];
	else if(time < SWING_END_M3)	qt[WAM_M3] = waveGeneSin(time-SWING_START_M3, SWING_TIME_M3, takeback_jnt_ang[WAM_M3], swing_end_jnt_ang[WAM_M3]);
	else	qt[WAM_M3] = swing_end_jnt_ang[WAM_M3];

	// expŒ^
	if(time < TAKEBACK_START_M4)	qt[WAM_M4] = prepare_qt[WAM_M4];
	else if(time < TAKEBACK_END_M4)	qt[WAM_M4] = waveGeneSin(time-TAKEBACK_START_M4, TAKEBACK_TIME_M4, prepare_qt[WAM_M4], takeback_jnt_ang[WAM_M4]);
	//else if(time < SWING_START_M4)	qt[WAM_M4] = takeback_jnt_ang[WAM_M4];
	else if(time < SWING_END_M4)	qt[WAM_M4] = takeback_jnt_ang[WAM_M4];
//	else if(time < SWING_END_M4)	qt[WAM_M4] = waveGeneExp(time-SWING_START_M4, SWING_TIME_M4/4, takeback_jnt_ang[WAM_M4], PI/2.5);
	//else if(time < SWING_END_M4)	qt[WAM_M4] = waveGeneExp(time-SWING_START_M4, SWING_TIME_M4/4, takeback_jnt_ang[WAM_M4], 3*PI/10);
	else	qt[WAM_M4] = swing_end_jnt_ang[WAM_M4];

//	qt[WAM_M1] = 0;
//	qt[WAM_M3] = 0;
//	qt[WAM_M4] = 0;
//	qt[WAM_M1] = prepare_qt[WAM_M1];
//	qt[WAM_M3] = prepare_qt[WAM_M3];
//	qt[WAM_M4] = prepare_qt[WAM_M4];

	// 2²
	if(time < TAKEBACK_START_M1)	qt[WAM_M2] = prepare_jnt_ang[WAM_M2];
	else if(time < TAKEBACK_END_M1)	qt[WAM_M2] = waveGeneSin(time-TAKEBACK_START_M1, TAKEBACK_TIME_M1, prepare_jnt_ang[WAM_M2], WAM_M2_JNT_ANG);
	else	qt[WAM_M2] = WAM_M2_JNT_ANG;

	//‰ŠúˆÊ’u‚ÅŒÅ’è ƒeƒXƒg—p
	//qt[WAM_M1] = prepare_qt[WAM_M1];
	//qt[WAM_M2] = prepare_qt[WAM_M2];
	//qt[WAM_M3] = prepare_qt[WAM_M3];
	//qt[WAM_M4] = prepare_qt[WAM_M4];

	// ƒoƒŒƒbƒgŠÖß‚É•ÏŠ·
	battingModelJnt2armJnt(ref_jnt_ang, dammy, dammy, qt, dqt, ddqt);

	return 0;
}

//////////////////////////////////////
// ƒA[ƒ€‰Šúİ’è
//////////////////////////////////////
int wam2AppSet(WAM *wam)
{
	int jnt;
//////////////////////////////////////
	wam->cons.da_limit = 1.0;		// 0.0 ` 1.0i‚±‚Ì’l‚ÉDA_LIMIT_WAM‚ğ‚©‚¯‚½DAw—ß‚ğo—Íj
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

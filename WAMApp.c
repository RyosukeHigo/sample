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
// �z�X�gPC�Ƃ̒ʐM�ϐ�
// [model]_usr.c ���Œ�`
///////////////////////////////
#ifndef MATLAB_MEX_FILE

#else

#endif

//�@batting_ball_count �̒l�Ə�Ԃ̃J�E���g�̑Ή�
enum {	
	BC_STRIKE = 1,		//�X�g���C�N
	BC_BALL,			//�{�[��
	BC_BALL_DZONE,		//�{�[���i�댯�̈�̂��߁j
	BC_BALL_INSWING,	//�{�[���i�X�E�B���O���Ƀ{�[���ɂȂ����j
};

// �\���̐錾
WAM		wam;


// �O���[�o�����W�ɂ�����A�[���ʒu
static HomoMat wam_base_homo ={{1.0, 0.0, 0.0},
		                        {0.0, 1.0, 0.0},
		                        {0.0, 0.0, 1.0},
								{0.0, 0.675, 0.0}};		// ��Ղ���1�����S�ʒu�܂ł̍���84.675cm, ��Ռ�����5cm, ���������ʂ����Օ\�ʂ܂ł�5.5cm(���̉��ʂɂ�葽�������Ă���)
// �������z=0�܂ł̍���90.175cm

// ����Q�C��
static WAMCtrlCoef wam_ctrl_coef = {{8.0, 8.0, 8.0, 8.0, 1000.0},				// Kp
									{0.0, 0.0, 0.0, 0.0, 0.0},					// Ti
									{120.0, 120.0, 80.0, 80.0, 3000.0},			//Td
									{0.0, 0.0, 0.0, 0.0, 0.0},					// Cf
									100};										// Kg
// �����p��
const static WAMJnt   prepare_jnt_ang = {-PI/2.0+0.1, PI/5.0, -3.0*PI/4.0, 7.0*PI/12.0}; 

int positionFlag; //�{�[�����ʂ肷�������ǂ����@20090618

//////////////////////////////////////////////////////
int least_square(double *x_coef, double *y_coef, double *obj_pos, int count, int flag)
{
	static double	sum_x = 0;
	static double	sum_y = 0;
	static double	sum_t = 0, sum_tx = 0, sum_ty = 0, sum_t2 = 0;
	static int		num = 0;
	double	denominator, time;

	// ���Z�b�g
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

	// ����
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
// �o�b�e�B���O
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
// �A�[�������ݒ�
//////////////////////////////////////
int wamAppSet(WAM *wam)
{
	int jnt;
//////////////////////////////////////
	wam->cons.da_limit = 1.0;		// 0.0 �` 1.0�i���̒l��DA_LIMIT_WAM��������DA�w�߂��o�́j
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

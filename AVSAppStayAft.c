#include <stdio.h>
#include <math.h>
#include <AVS.h>
#include "App.h"

// �\���̐錾
AVSALL	avs;

// �O���[�o�����W�ɂ�����AVS�ʒu
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
//										{1.438, 0.4, 0.148}},	// AVS���t���ʒu�E�E�A�[�����̒�Ռ�����4��
//										{1.538, 0.4, 0.148}},	// AVS���t���ʒu�E�E�A�[�����̒�Ռ�����8��
										//{1.588, 0.4, 0.148}},	// AVS���t���ʒu�E�E�A�[�����̒�Ռ�����10��
										//{1.588, 0.4, 0.348}},	// �l�����p�ɍ������� 2010.06.02
										{1.665, 0.4, 0.148}},	// WAM�̒�Ղ����ꂽ�@2010.11.15
										{{0.0, -1.0, 0.0},
										{1.0, 0.0, 0.0},
										{0.0, 0.0, 1.0},
//										{1.438, -0.4, 0.148}}};		// ��Ղ�x�������̒[��x=0.75
//										{1.538, -0.4, 0.148}}};		// ��Ղ�x�������̒[��x=0.75
										//{1.588, -0.4, 0.148}}};		// ��Ղ�x�������̒[��x=0.75
										//{1.588, -0.4, 0.348}}};		// �l�����p�ɍ������� 2010.06.02
										{1.665, -0.4, 0.148}}};	// WAM�̒�Ղ����ꂽ�@2010.11.15
// �������Օ\�ʂ܂ō���54cm, �₮��30cm, �`���g���܂ł̍���21cm
// ������`���g���܂ł̍���105cm - ������A�[�����_�܂ł̍���90.175cm = 14.825cm
// ��Ւ[�܂�75cm + ��Ւ[����AVS�h�U��r�܂�52cm �h�U��r����AVS��Ւ[ -0.9cm AVS��Ւ[����1�ڂ̌��܂�+3.7cm�@�₮����t���ʒu+7.5cm�@�₮����t��������AVS�����S+6.5 = 143.8cm


// ����Q�C��
#if 0

static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 600.0, 600.0, 600.0},    // Kp
									   {0.0, 0.0, 0.0, 0.0},    // Ti
									   {14000.0, 20000.0, 20000.0, 20000.0},      // Td
									   {7.0, 7.0, 10.0, 10.0},     // IcKp�F�r�W���A���T�[�{�Q�C��
									   50.0};                 // Kg


#else	// 20090325
//static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 600.0, 600.0, 600.0},    // Kp
//									   {0.0, 0.0, 0.0, 0.0},    // Ti
//									   {12000.0, 20000.0, 20000.0, 20000.0},      // Td
//									   {7.0, 7.0, 10.0, 10.0},     // IcKp�F�r�W���A���T�[�{�Q�C��
//									   50.0};                 // Kg
// 20090721
//static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 600.0, 400.0, 400.0},    // Kp
//									   {0.0, 0.0, 0.0, 0.0},    // Ti
//									   {12000.0, 20000.0, 14000.0, 14000.0},      // Td
//									   {7.0, 7.0, 10.0, 10.0},     // IcKp�F�r�W���A���T�[�{�Q�C��
//									   50.0};                 // Kg
// 20091113
//static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 500.0, 400.0, 400.0},    // Kp
//									   {0.0, 0.0, 0.0, 0.0},    // Ti
//									   {12000.0, 15000.0, 14000.0, 14000.0},      // Td
//									   {6.5, 6.5, 6.5, 6.5},     // IcKp�F�r�W���A���T�[�{�Q�C��
//									   50.0};                 // Kg

// 20091114
//static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 500.0, 400.0, 400.0},    // Kp
//									   {0.0, 0.0, 0.0, 0.0},    // Ti
//									   {12000.0, 15000.0, 14000.0, 14000.0},      // Td
//									   {5.5, 6.5, 6.5, 6.5},     // IcKp�F�r�W���A���T�[�{�Q�C��
//									   40.0};                 // Kg
//20091202
static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 500.0, 400.0, 400.0},    // Kp
									   {0.0, 0.0, 0.0, 0.0},    // Ti
									   {12000.0, 15000.0, 14000.0, 14000.0},      // Td
									   {6.5, 6.5, 6.5, 6.5},     // IcKp�F�r�W���A���T�[�{�Q�C��
									   50.0};                 // Kg

//static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 500.0, 500.0, 400.0},    // Kp
//									   {0.0, 0.0, 0.0, 0.0},    // Ti
//									   {12000.0, 15000.0, 14500.0, 14000.0},      // Td
//									   {7.0, 7.0, 8.0, 8.0},     // IcKp�F�r�W���A���T�[�{�Q�C��
//									   50.0};                 // Kg

#endif
// �����p��
//static AVSJntAll	avs_prepare_jnt_ang = {PI/4, -PI/4, PI/4, PI/4};		// �X���[�C���O�A�[������(�l�������鎞)
//static AVSJntAll	avs_prepare_jnt_ang = {PI/5, -PI/3, PI/5, PI/3};		// �X���[�C���O�A�[������(WAM2�������鎞)
//static AVSJntAll	avs_prepare_jnt_ang = {PI/5, -PI/3+0.07, PI/5, PI/3-0.07};		// �X���[�C���O�A�[������(WAM2�������鎞)
//  static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/12, -PI/3, PI/5-PI/12, PI/3};		// �X���[�C���O�A�[������(WAM2�������鎞) 091027
//  static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/24, -PI/3, PI/5-PI/24, PI/3};		// �X���[�C���O�A�[������(WAM2�������鎞)
//  static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/12, -PI/3, PI/5-PI/12, PI/3};		// �X���[�C���O�A�[������(WAM2�������鎞)
//static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/24, PI/3, PI/5-PI/24, -PI/3};		// �o�b�e�B���O�A�[������(�l��������i�f��������j��)
//  static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/12+0.1, -PI/3, PI/5-PI/12+0.1, PI/3};		// �X���[�C���O�A�[������(WAM2�������鎞) 100728 ����
  //static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/12+0.03, -PI/3, PI/5-PI/12+0.03, PI/3};		// �X���[�C���O�A�[������(WAM2�������鎞) 110418 ����
//  static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/12+0.03, -PI/3-0.1521, PI/5-PI/12+0.03-0.0259, PI/3+0.1652+0.0264};		// 111109 ����
  static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/12, -PI/3-0.1721, PI/5-PI/12+0.03, PI/3+0.2116};		// 120326 ����



#ifndef MATLAB_MEX_FILE
extern int positionFlag; //�{�[�����ʂ�߂������ǂ��� 
#else
  int positionFlag;
#endif

//static AVSJntAll avs_last_jnt_ang = {PI/4, -PI/3, PI/4, PI/3}; //avs�̍ŏI�p��
//static AVSJntAll avs_last_jnt_ang = {PI/3, -PI/3, PI/3, PI/3}; //avs�̍ŏI�p��
static AVSJntAll avs_last_jnt_ang = {PI/3, PI/3, PI/3, -PI/3}; //avs�̍ŏI�p��
//////////////////////////////////////////////////////
int avsTrajApp(AVSJntAll ref_jnt_ang, double time)
{
	int jnt;
/////////////////////////////
// �Ώۂ����������Ƃ��̋O��
//
	//�����Ă��������������͏����ʒu�ɖ߂邪�C
	//������x��������̏ꍇ��1�t���[���O�̈ʒu�ɗ��܂�D
	//ref_jnt_ang�͂��̊֐��ɓ���O��1�t���[���O�̒l�ŏ��������Ă邽��
	//����������Ȃ���΁C1�O�̊֐ߊp�ɂȂ�D
/////////////////////////////
	if(positionFlag == 0){
		for(jnt=0;jnt<AVS_JNT_ALL;jnt++){
			ref_jnt_ang[jnt] = avs_prepare_jnt_ang[jnt];
		}
	}
		return 0;
  }

////////////////////////
// �r�W���A���T�[�{
////////////////////////
#if 1
int avsVisualTrajApp(AVSImgCenAll ref_img_center, const AVSImgCenAll img_center, double time)
{
	// �Ώۂ��摜���S�ɂ���悤�ɐ���
	ref_img_center[AVS1_PAN] = 0.0;
	ref_img_center[AVS1_TILT] = 0.0;
	ref_img_center[AVS2_PAN] = 0.0;
	ref_img_center[AVS2_TILT] = 0.0;
	return 0;
}

#else
////////////////////////
// �o�b�e�B���O�p�r�W���A���T�[�{
////////////////////////
int avsVisualTrajApp(AVSImgCenAll ref_img_center, const AVSImgCenAll img_center, double time)
{
#define NON_MOVABLE     0
#define MOVABLE         1
#define APPROACH_TIME 100		// [ms]

    static int m_flag[AVS_NUM], incount[AVS_NUM];
    static double init_y[AVS_NUM];

    // ������
    if(time < 0.0025){ incount[AVS1] = 0; m_flag[AVS1] = NON_MOVABLE;}
	if(img_center[IMG1_VISIBLE] > 0){
		/////// x���W ////////
		if(m_flag[AVS1] == NON_MOVABLE){
			ref_img_center[AVS1_PAN] = avs_prepare_jnt_ang[0];
			if(img_center[AVS1_PAN] > 0) m_flag[AVS1] = MOVABLE;
		}else{
			ref_img_center[AVS1_PAN] = 0.0;
		}
		/////// y���W ////////
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

    // ������
    if(time < 0.0025){ incount[AVS2] = 0; m_flag[AVS2] = NON_MOVABLE;}
	if(img_center[IMG2_VISIBLE] > 0){
		/////// x���W ////////
		if(m_flag[AVS2] == NON_MOVABLE){
			ref_img_center[AVS2_PAN] = avs_prepare_jnt_ang[0];
			if(img_center[AVS2_PAN] < 0) m_flag[AVS2] = MOVABLE;
		}else{
			ref_img_center[AVS2_PAN] = 0.0;
		}
		/////// y���W ////////
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
// �A�N�e�B�u�r�W���������ݒ�
//////////////////////////////////////
int avsAppSet(AVSALL *avs)
{
	int jnt, num;
//////////////////////////////////////
	avs->cons.da_limit = 0.8;		// 0.0 �` 1.0�i���̒l��DA_LIMIT_AVS��������DA�w�߂��o�́j
//////////////////////////////////////
	avs->base_homo = avs_base_homo;
	avs->ctrl_coef = &avs_ctrl_coef;
	avs->prepare_jnt_ang = avs_prepare_jnt_ang;
	return 0;
}

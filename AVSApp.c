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
static HomoMat avs_base_homo[AVS_NUM] = { { { 0.0, -1.0, 0.0 },
											{ 1.0, 0.0, 0.0 },
											{ 0.0, 0.0, 1.0 },
//										{1.438, 0.4, 0.148}},	// AVS���t���ʒu�E�E�A�[�����̒�Ռ�����4��
//										{1.538, 0.4, 0.148}},	// AVS���t���ʒu�E�E�A�[�����̒�Ռ�����8��
										//{1.588, 0.4, 0.148}},	// AVS���t���ʒu�E�E�A�[�����̒�Ռ�����10��
										//{1.588, 0.4, 0.348}},	// �l�����p�ɍ������� 2010.06.02
										//{1.665, 0.4, 0.148}},	// WAM�̒�Ղ����ꂽ�@2010.11.15
										{ 1.604, 0.4, 0.177 } },	// �L�����u���[�V�����@2017.02.15
										{ { 0.0, -1.0, 0.0 },
										{ 1.0, 0.0, 0.0 },
										{ 0.0, 0.0, 1.0 },
//										{1.438, -0.4, 0.148}}};		// ��Ղ�x�������̒[��x=0.75
//										{1.538, -0.4, 0.148}}};		// ��Ղ�x�������̒[��x=0.75
										//{1.588, -0.4, 0.148}}};		// ��Ղ�x�������̒[��x=0.75
										//{1.588, -0.4, 0.348}}};		// �l�����p�ɍ������� 2010.06.02
										//{1.665, -0.4, 0.148}}};	// WAM�̒�Ղ����ꂽ�@2010.11.15
										{ 1.604, -0.4, 0.177 } } };	// �L�����u���[�V�����@2017.02.15
// �������Օ\�ʂ܂ō���54cm, �₮��30cm, �`���g���܂ł̍���21cm
// ������`���g���܂ł̍���105cm - ������A�[�����_�܂ł̍���90.175cm = 14.825cm
// ��Ւ[�܂�75cm + ��Ւ[����AVS�h�U��r�܂�52cm �h�U��r����AVS��Ւ[ -0.9cm AVS��Ւ[����1�ڂ̌��܂�+3.7cm�@�₮����t���ʒu+7.5cm�@�₮����t��������AVS�����S+6.5 = 143.8cm

double cur_jnt_ang[2];//save the current joints

// ����Q�C��
#if 0

static AVSCtrlCoefAll	avs_ctrl_coef = { { 400.0, 600.0, 600.0, 600.0 },    // Kp
{ 0.0, 0.0, 0.0, 0.0 },    // Ti
{ 14000.0, 20000.0, 20000.0, 20000.0 },      // Td
{ 7.0, 7.0, 10.0, 10.0 },     // IcKp�F�r�W���A���T�[�{�Q�C��
50.0 };                 // Kg


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
//static AVSCtrlCoefAll	avs_ctrl_coef = {{400.0, 500.0, 400.0, 400.0},    // Kp
//									   {0.0, 0.0, 0.0, 0.0},    // Ti
//									   {12000.0, 15000.0, 14000.0, 14000.0},      // Td
//									   {6.5, 6.5, 6.5, 6.5},     // IcKp�F�r�W���A���T�[�{�Q�C��	//20151124
//									   50.0 };                 // Kg	//	20151124
//										30.0 };                 // Kg	//	20151124
//Photron
//static AVSCtrlCoefAll	avs_ctrl_coef = { { 400.0, 400.0, 400.0, 400.0 },    // Kp
//										{ 0.0, 0.0, 0.0, 0.0 },    // Ti
//										{ 14000.0, 14000.0, 14000.0, 14000.0 },      // Td
//										{ 3, 3, 4.5, 4.5 },     // IcKp�F�r�W���A���T�[�{�Q�C��	//20151124
//										{ 33000.0, 32000.0, 33000.0, 36000.0 }, //IcTd: �r�W���A���T�[�{D�Q�C��
//										50.0 };                 // Kg	//	20151124
//20160615 ��̃p�����^�Ŋ֐ߊp���䎞�ɐU��
//static AVSCtrlCoefAll	avs_ctrl_coef = { { 300.0, 300.0, 300.0, 300.0 },    // Kp
//										{ 0.0, 0.0, 0.0, 0.0 },    // Ti
//										{ 10000.0, 10000.0, 10000.0, 10000.0 },      // Td
//										{ 2, 2, 2, 2 },     // IcKp�F�r�W���A���T�[�{�Q�C��	//20151124
//										{ 16000.0, 19000.0, 14000.0, 16000.0 }, //IcTd: �r�W���A���T�[�{D�Q�C��
//										80.0 };                 // Kg	//	20151124
//20160616 
//static AVSCtrlCoefAll	avs_ctrl_coef = { { 400.0, 400.0, 400.0, 400.0 },    // Kp
//										{ 0.0, 0.0, 0.0, 0.0 },    // Ti
//										{ 14000.0, 15000.0, 14000.0, 14000.0 },      // Td
//										{ 4, 6, 2, 2 },     // IcKp�F�r�W���A���T�[�{�Q�C��	//20151124
//										{ 33000.0, 47000.0, 14000.0, 14000.0 }, //IcTd: �r�W���A���T�[�{D�Q�C��
//										50.0 };                 // Kg	//	20151124
//20160801 �����^�������Y
//static AVSCtrlCoefAll	avs_ctrl_coef = { { 400.0, 400.0, 400.0, 400.0 },    // Kp
//										{ 0.0, 0.0, 0.0, 0.0 },    // Ti
//										{ 14000.0, 15000.0, 14000.0, 14000.0 },      // Td
//										{ 8.5, 8.5, 2, 2 },     // IcKp�F�r�W���A���T�[�{�Q�C��	//20151124
//										{ 30000.0, 30000.0, 14000.0, 14000.0 }, //IcTd: �r�W���A���T�[�{D�Q�C��
//										50.0 };                 // Kg	//	20151124
//20160928 �����Y����
//static AVSCtrlCoefAll	avs_ctrl_coef = { { 400.0, 400.0, 400.0, 400.0 },    // Kp
//										{ 0.0, 0.0, 0.0, 0.0 },    // Ti
//										{ 14000.0, 15000.0, 14000.0, 14000.0 },      // Td
//										{ 8.5, 8.5, 8.5, 8.5 },     // IcKp�F�r�W���A���T�[�{�Q�C��	//20151124
//										{ 30000.0, 30000.0, 30000.0, 30000.0 }, //IcTd: �r�W���A���T�[�{D�Q�C��
//										50.0 };                 // Kg	//	20151124
//20170111 �p�����^���� �`���g���U��
//static AVSCtrlCoefAll	avs_ctrl_coef = { { 400.0, 400.0, 400.0, 400.0 },    // Kp
//{ 0.0, 0.0, 0.0, 0.0 },    // Ti
//{ 14000.0, 15000.0, 14000.0, 14000.0 },      // Td
//{ 8.5, 8.5, 8.5, 8.5 },     // IcKp�F�r�W���A���T�[�{�Q�C��	//20151124
//{ 30000.0, 30000.0, 30000.0, 30000.0 }, //IcTd: �r�W���A���T�[�{D�Q�C��
//50.0 };                 // Kg	//	20151124

//20170220 �p�����^���� �`���g���U��
static AVSCtrlCoefAll	avs_ctrl_coef = { { 400.0, 400.0, 400.0, 400.0 },    // Kp
{ 0.0, 0.0, 0.0, 0.0 },    // Ti
{ 14000.0, 15000.0, 14000.0, 14000.0 },      // Td
{ 7.5, 8.5, 7.5, 8.5 },     // IcKp�F�r�W���A���T�[�{�Q�C��	//20151124
{ 30000.0, 30000.0, 30000.0, 30000.0 }, //IcTd: �r�W���A���T�[�{D�Q�C��
50.0 };                 // Kg	//	20151124

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
// static AVSJntAll	avs_prepare_jnt_ang = {PI/5-PI/12, -PI/3-0.1721, PI/5-PI/12+0.03, PI/3+0.2116};		// 120326 ����
static AVSJntAll	avs_prepare_jnt_ang = { PI / 5 - PI / 12, -PI / 3 - 0.2161, PI / 5 - PI / 12 + 0.01, PI / 3 + 0.2116 };		// 120810 ����
//static AVSJntAll	avs_prepare_jnt_ang = { PI / 5 - PI / 12 + 0.01, PI / 3 + 0.04, PI / 5 - PI / 12 + 0.01, -PI / 3 - 0.25 };		// 2012.3.12 �o�b�e�B���O������

//static AVSJntAll	avs_prepare_jnt_ang = { 0,0,0,0 };		// 170203 0�_����


#ifndef MATLAB_MEX_FILE
extern int positionFlag; //�{�[�����ʂ�߂������ǂ��� 
#else
int positionFlag = 0;
#endif

//static AVSJntAll avs_last_jnt_ang = {PI/4, -PI/3, PI/4, PI/3}; //avs�̍ŏI�p��
//static AVSJntAll avs_last_jnt_ang = {PI/3, -PI/3, PI/3, PI/3}; //avs�̍ŏI�p��
static AVSJntAll avs_last_jnt_ang = { PI / 3, PI / 3, PI / 3, -PI / 3 }; //avs�̍ŏI�p��
//////////////////////////////////////////////////////
int avsTrajApp(AVSJntAll ref_jnt_ang, AVSJntAll past_jnt_ang, AVSJntAll jnt_ang, double time)
{
	int jnt;
	static int trackFlag[AVS_JNT_ALL] = { 0, 0, 0, 0 };
	double rate[2];
	double grad[2];
	double lTime[2];
	double qTime[2];
	int crd;

	////////////////////////
	// ���n��O��
	////////////////////////
#if	0
#define AVS_TRAJ_RAD PI/4.0
#define AVS_TRAJ_RAD2 PI/6.0
#define AVS_TRAJ_RATE 1.0
	ref_jnt_ang[AVS1_PAN] = -AVS_TRAJ_RAD*(1.0 - cos(2.0*PI*time*AVS_TRAJ_RATE)) + avs_prepare_jnt_ang[AVS_M1];
	ref_jnt_ang[AVS1_TILT] = AVS_TRAJ_RAD2*(1.0 - cos(2.0*PI*time*AVS_TRAJ_RATE)) + avs_prepare_jnt_ang[AVS_M2];
	ref_jnt_ang[AVS2_PAN] = AVS_TRAJ_RAD*(1.0 - cos(2.0*PI*time*AVS_TRAJ_RATE)) + avs_prepare_jnt_ang[AVS_M3];
	ref_jnt_ang[AVS2_TILT] = -AVS_TRAJ_RAD2*(1.0 - cos(2.0*PI*time*AVS_TRAJ_RATE)) + avs_prepare_jnt_ang[AVS_M4];
#endif
	if (time < 0.002){
		for (jnt = 0; jnt < AVS_JNT_ALL; jnt++){
			trackFlag[jnt] = 0;
		}
	}

	/////////////////////////////
	// �Ώۂ����������Ƃ��̋O��
	/////////////////////////////
	for (jnt = 0; jnt < AVS_JNT_ALL; jnt++)	ref_jnt_ang[jnt] = avs_prepare_jnt_ang[jnt];

	//����������́C���������ʒu�őҋ@(���Ԃ�0.5�͂��܂�Ӗ��Ȃ� ���炩�̃g���K�ɕύX���ׂ�)
	if (time > 0.3){
		for (jnt = 0; jnt < AVS_JNT_ALL; jnt++)	{
			if (trackFlag[jnt] == 1){
				ref_jnt_ang[jnt] = jnt_ang[jnt];
			}
			else{
				if (fabs(jnt_ang[jnt] - avs_prepare_jnt_ang[jnt]) > 0.2){
					trackFlag[jnt] = 1;
				}
			}
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
	if (time < 0.0025){ incount[AVS1] = 0; m_flag[AVS1] = NON_MOVABLE; }
	if (img_center[IMG1_VISIBLE] > 0){
		/////// x���W ////////
		if (m_flag[AVS1] == NON_MOVABLE){
			ref_img_center[AVS1_PAN] = avs_prepare_jnt_ang[0];
			if (img_center[AVS1_PAN] > 0) m_flag[AVS1] = MOVABLE;
		}
		else{
			ref_img_center[AVS1_PAN] = 0.0;
		}
		/////// y���W ////////
		if (incount[AVS1] == 0){
			init_y[AVS1] = img_center[AVS1_TILT];
			ref_img_center[AVS1_TILT] = init_y[AVS1];
		}
		else if (incount[AVS1] < APPROACH_TIME){
			ref_img_center[AVS1_TILT] = (APPROACH_TIME - incount[AVS1])*init_y[AVS1] / APPROACH_TIME;
		}
		else{
			ref_img_center[AVS1_TILT] = 0.0;
		}
		incount[AVS1]++;
	}

	// ������
	if (time < 0.0025){ incount[AVS2] = 0; m_flag[AVS2] = NON_MOVABLE; }
	if (img_center[IMG2_VISIBLE] > 0){
		/////// x���W ////////
		if (m_flag[AVS2] == NON_MOVABLE){
			ref_img_center[AVS2_PAN] = avs_prepare_jnt_ang[0];
			if (img_center[AVS2_PAN] < 0) m_flag[AVS2] = MOVABLE;
		}
		else{
			ref_img_center[AVS2_PAN] = 0.0;
		}
		/////// y���W ////////
		if (incount[AVS2] == 0){
			init_y[AVS2] = img_center[AVS2_TILT];
			ref_img_center[AVS2_TILT] = init_y[AVS2];
		}
		else if (incount[AVS2] < APPROACH_TIME){
			ref_img_center[AVS2_TILT] = (APPROACH_TIME - incount[AVS2])*init_y[AVS2] / APPROACH_TIME;
		}
		else{
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

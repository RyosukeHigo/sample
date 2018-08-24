#ifndef APP_H
#define APP_H

#include "Data.h"
#include "SysDef.h"
#if SYSTEM_HAND
#include <Hand.h>
#endif
#if SYSTEM_WAM
#include <WAM.h>
#endif
#if SYSTEM_AVS
#include <AVS.h>
#endif
#if SYSTEM_XYZ
#include <XYZstage.h>
#endif //SYSTEM_XYZ

// ���Ԑݒ�
#define START_TIME	(5.0)       // �����p�����珀���p���ւ̎���
#define END_TIME	(5.0)       // �I���p�����珉���p���ւ̎���
#define MOTION_TIME	(15.0)      // ���쎞��
#define ALL_APP_TIME    (START_TIME+MOTION_TIME+END_TIME)

// �����p��
#define WAM_M1_INIT_JNT_ANG		0.0
#define WAM_M2_INIT_JNT_ANG		-2.00283
#define WAM_M3_INIT_JNT_ANG		0.0
#define WAM_M4_INIT_JNT_ANG		2.50422
#define WAM_M5_INIT_JNT_ANG		0.0

//�V�~�����[�V���������@�������̑I��
#define SIM_OR_EXP (0)			//1:�V�~�����[�V�����C0:���@����

// �n���h
#if SYSTEM_HAND
int handTrajApp(HANDJnt ref_jnt_ang, DATA *data, double time, double *eoCam);
//int handTrajApp(HANDJnt ref_jnt_ang, DATA *data, double time, double *eoCam, double *output);
int handAppSet(HAND *hand);
#endif
// �A�[��
#if SYSTEM_WAM
int wamTrajApp(WAMJnt ref_jnt_ang, DATA *data, double time, int appNum);
int wamAppSet(WAM *wam);
int wam2TrajApp(WAMJnt ref_jnt_ang, DATA *data, double time);
int wam2AppSet(WAM *wam);
#endif
// �A�N�e�B�u�r�W����
#if SYSTEM_AVS
int avsTrajApp(AVSJntAll ref_jnt_ang, AVSJntAll past_jnt_ang, AVSJntAll jnt_ang, double time);
int avsVisualTrajApp(AVSImgCenAll ref_img_center, const AVSImgCenAll img_center, double time);
int avsAppSet(AVSALL *avs);
#endif
// XYZ�X�e�[�W
#if SYSTEM_XYZ
int xyzTrajApp(XYZJnt ref_jnt_pos, XYZJnt data, double *obj, double time, double *sensor, double *camera);
int xyzAppSet(XYZ *xyz);
#endif //SYSTEM_XYZ

//�O������̃f�[�^
#define Ethernet_RECV_NUM (14) //�C�[�T�l�b�g��M�f�[�^���@�摜����PC����
#define Ethernet_SEND_NUM (12) //�C�[�T�l�b�g���M�f�[�^���@�摜����PC����
#define SENSOR_DATA_NUM (18) //�Z���T�f�[�^��

#endif

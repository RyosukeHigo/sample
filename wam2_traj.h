#ifndef WAM2_TRAJ_H_INCLUDED
#define WAM2_TRAJ_H_INCLUDED

int rangeSelector(const double z);	//�ߋ��_��z���W���獇������O����I��

//�X�P�[���ϊ����l�������֐ߊp�O������
double waveGeneAxis1(const double motion_time, double scale, int prevMotion); //1��
double waveGeneAxis3(const double motion_time, const double catch_jnt, const double z_pos, double scale); //3��
double waveGeneAxis4(const double motion_time, const double catch_jnt, const double z_pos, double scale); //4��

//�����֐ߊp
#define PREPARE_WAM2_AXIS1 (1.1254) //WAM2��1���̏����֐ߊp
//const static WAMJnt	prepare_jnt_ang = {0.6283, PI/5, 0.0, PI/12, 0.0};
//const static WAMJnt	prepare_jnt_ang = {1.1254, PI/5, 0.0, PI/12, 0.0}; //2012.12.19 �Ƃ肠����1�������萔�ŕς���l�ɂ����D
//int positionFlag;	//�{�[�����ʂ肷�������ǂ����@20090618 �Ƃ肠����wam2App.c�֖߂�

// �ߋ��܂ł̎���
#define BEFORE_CATCHING_TIME		(0.23)	//0.3690
#define BEFORE_CATCHING_TIME_FULL	(0.369) //1���̍ő�ߋ��O����
#define BEFORE_CATCHING_TIME_AX34	(0.169) //3,4���̕ߋ��O�̎���
#define DELAY_TIME					(BEFORE_CATCHING_TIME - BEFORE_CATCHING_TIME_AX34) //3,4����1,2���ɑ΂��Ēx��鎞�ԁi�b�j
#define SHORTCUT_TIME				(BEFORE_CATCHING_TIME_FULL - BEFORE_CATCHING_TIME) //�ߋ��O���Ԃ̍ő厞�Ԃɔ�ׂĉ��b�Z����
#define SWING_TIME					(1.0) //1���̌��̓���̓��쎞�ԁi�b�j
#define SWING_TIME_AX34				(0.7) //3,4���̓��쎞��(�b)
#define MIN_X						(3.9)  //�p�ӂ����֐��̈�ԉ�
#define MAX_X						(3.3)  //�p�ӂ����֐��̈�ԉ�
#define SWING_OFFSET				(0)//(0.125)//(0.03)	// �{�[���̌����l���̂��߂̃I�t�Z�b�g
#define AFTER_CATCHING_TIME		    (0.60) //�ߋ���ŏI�ʒu�ɖ߂�܂ł̎���

#define MAX_Z	(0.70)	//�ߋ��_�̋��e�ő�z���W
#define MIN_Z	(0.35)	//�ߋ��_�̋��e�ŏ�z���W			

#define SCALE_BUFFER_COUNT (5)	//�X�P�[���ϊ���C�ߋ��_�Œʏ�̑��x�ŕߋ����邽�߂̗]��J�E���g��
#define MIN_SCALE_APPLY_COUNT (15+SCALE_BUFFER_COUNT)	//�X�P�[���ϊ���K�p����ŏ��J�E���g��

#define AFTER_LAUNCH_TIME (0.546) //���˂���ߋ��܂ł̕��ϓI�Ȏ���

#endif /*WAM2_TRAJ_H_INCLUDED*/
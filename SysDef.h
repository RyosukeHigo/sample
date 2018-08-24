#ifndef SYSDEF_H
#define SYSDEF_H

//////////////////////////////////////////////////
// �V�X�e���g�p�ݒ�
// �g�p����Ƃ��� 1 , �g�p���Ȃ��Ƃ��� 0 ��ݒ�
// HAND1: 02�n���h
// HAND2: �o���b�g�p�n���h
// WAM1: �o�b�e�B���O�A�[��
// WAM2: �X���[�C���O�A�[��
//////////////////////////////////////////////////
#define SYSTEM_HAND1	0		// �O�Q�n���h
#define SYSTEM_HAND2	1		// �o���b�g�p�n���h
#define SYSTEM_WAM1     0		// �o�b�e�B���O�p�A�[��
#define SYSTEM_WAM2     0		// �X���[�C���O�p�A�[��
#define SYSTEM_XYZ  	1
#define SYSTEM_AVS      0
#define SYSTEM_FORCE	0
#define SYSTEM_HAND_WAM_CONNECT		0		// �n���h���A�[���ɓ���
#define SYSTEM_HAND		(SYSTEM_HAND1 || SYSTEM_HAND2)		// �n���h���ǂ��炩�g���ꍇ
#define SYSTEM_WAM      (SYSTEM_WAM1 || SYSTEM_WAM2)		// �A�[�����ǂ��炩�g���ꍇ
#define SYSTEM_WAM_NUM  (SYSTEM_WAM1 + SYSTEM_WAM2)		// �A�[�����g���䐔

// ENC
#define NORMALIZE_ENC	    pow(2.0,31) //ds3002
#define NORMALIZE_ENC_STAGE pow(2.0,23) //ds3001
// DA (scan table number ��4�܂�)
#define TBL_HAND 1
#define TBL_XYZ  2
#define TBL_WAM  3
#define TBL_SERVO  3
#define TBL_AVS  4
#define DA_LIMIT_HAND	1.0
#define DA_LIMIT_WAM	0.45		// �}�j���A�����0.42(4.2V)�܂�
#define DA_LIMIT_AVS	1.0
#define DA_LIMIT_XYZ	0.9

#endif

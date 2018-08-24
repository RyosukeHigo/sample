// Usage:
//
//    predict3D_init();                // ������
//    ...
//    predict3D_start(obj_pos, count); // �\���J�n
//    loop{
//          if(obj_pos is observed) predict3D(obj_pos_predcit, count_bias, obj_pos, count, interval);
//          else                    predict3D_calc(obj_pos_predcit, count_bias, count, interval);
//    }
//   predict3D_exit();                 // �I������

#ifndef PREDICT_3D_H
#define PREDICT_3D_H
#include <Kine.h>

//////////////////////////////////////////////////
// ������
//////////////////////////////////////////////////
void predict3D_init(void);

//////////////////////////////////////////////////
// �\���J�n
//////////////////////////////////////////////////
void predict3D_start(Vector3D obj_pos,  // �Ώۂ̈ʒu
		     int      count);   // �J�n���̃��[�v�J�E���^  

//////////////////////////////////////////////////
// �ΏۋO���̃p�����[�^�̍X�V�ƋO���\�� 
// �Ԓl: obj_pos_predict
//     ���[�v�J�E���^�� (count_bias + count)�̎���
//     �ΏۋO�� obj_pos_predict ��\������D
//////////////////////////////////////////////////
void predict3D(Vector3D obj_pos_predict,  // �Ώۂ̗\���ʒu
	       int      count_bias,       // �\�����_�̃J�E���^�l - ���J�E���^�l
	       Vector3D obj_pos,          // �����_�ł̑Ώۂ̈ʒu
	       int      count,            // �����_�ł̃��[�v�J�E���^
	       double    interval);        // �T�C�N���^�C���i�ʏ� 0.001[s]�j

//////////////////////////////////////////////////
// �ΏۋO���̗\�� �i�O���p�����[�^�̍X�V���s��Ȃ��j
// �Ԓl: obj_pos_predict
//     ���[�v�J�E���^�� (count_bias + count)�̎���
//     �ΏۋO�� obj_pos_predict ��\������D
//////////////////////////////////////////////////
void predict3D_calc(Vector3D obj_pos_predict,   // �Ώۂ̗\���ʒu
		    int      count_bias,        // �\�����_�̃J�E���^�l - ���J�E���^�l
		    int      count,             // �����_�ł̃��[�v�J�E���^
		    double    interval);         // �T�C�N���^�C���i�ʏ� 0.001[s]�j

//////////////////////////////////////////////////
// �ΏۋO���̃p�����[�^�̍X�V�ƋO���p�����[�^�̎擾
// �Ԓl: a[6]
//////////////////////////////////////////////////
void predict_trajectory(Vector3D obj_pos,
		  double a[6],
		  int count,
		  double interval);

//////////////////////////////////////////////////
// �I������
//////////////////////////////////////////////////
void predict3D_exit(void);


//////////////////////////////////////////////////
// �O���p�����[�^�̎擾
// �Ԓl: a[6]
// �ΏۋO��obj_pos_predict��
//    obj_pos_predict[0] = a[3]*t*t + a[0]*t + obj_pos_init[0];
//    obj_pos_predict[1] = a[4]*t*t + a[1]*t + obj_pos_init[1];
//    obj_pos_predict[2] = a[5]*t*t + a[2]*t + obj_pos_init[2];
// �ŗ\�������Dobj_pos_init �͗\���J�n���_�ł̑Ώۂ̈ʒu, t�͎��ԁD
//////////////////////////////////////////////////
void predict3D_get(double a[6]);

#endif

#include <stdio.h>
#include "Data.h"

//////////////////////////////////////////
/**
* @brief DATA�ϐ��ւ̊i�[�֐�
*
* @param[out] data	DATA�ϐ�
* @param[in] iput	�i�[������͕ϐ�(double)
* @return �����Ȃ�0��Ԃ�
*/
int storeInDATA(DATA *data, double *input){

	//0�����[�����g�͂Ƃ肠����0����
	data->cam.img1_mom0 = 0;
	data->cam.img2_mom0 = 0;

	data->cam.img1_state = (int)input[0];
	data->cam.img2_state = (int)input[1];
	data->cam.img1_mom1_x = input[2];
	data->cam.img1_mom1_y = input[3];
	data->cam.img2_mom1_x = input[4];
	data->cam.img2_mom1_y = input[5];

	data->dim3.obj_state = (int)input[6];
	data->dim3.obj_pos_x = input[7];
	data->dim3.obj_pos_y = input[8];
	data->dim3.obj_pos_z = input[9];

	return 0;
}


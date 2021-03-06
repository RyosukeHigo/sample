#include <stdio.h>
#include "Data.h"

//////////////////////////////////////////
/**
* @brief DATA変数への格納関数
*
* @param[out] data	DATA変数
* @param[in] iput	格納する入力変数(double)
* @return 成功なら0を返す
*/
int storeInDATA(DATA *data, double *input){

	//0次モーメントはとりあえず0を代入
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


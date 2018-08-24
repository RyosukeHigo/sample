#include <stdio.h>
#include "Data.h"

//////////////////////////////////////////
/**
* @brief DATA•Ï”‚Ö‚ÌŠi”[ŠÖ”
*
* @param[out] data	DATA•Ï”
* @param[in] iput	Ši”[‚·‚é“ü—Í•Ï”(double)
* @return ¬Œ÷‚È‚ç0‚ð•Ô‚·
*/
int storeInDATA(DATA *data, double *input){

	//0ŽŸƒ‚[ƒƒ“ƒg‚Í‚Æ‚è‚ ‚¦‚¸0‚ð‘ã“ü
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


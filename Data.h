#ifndef _INC_DATA
#define _INC_DATA

#include <DataDef.h>

//////////////////////////
// ‰æ‘œî•ñ
//////////////////////////
typedef struct{
	int		img1_state;
	int     img1_mom0;
	double	img1_mom1_x;
	double	img1_mom1_y;
	int     img2_state;
	int     img2_mom0;
	double	img2_mom1_x;
	double	img2_mom1_y;
}DataCam;

//////////////////////////
// 3ŸŒ³î•ñ
//////////////////////////
typedef struct{
	int		obj_state;
	double	obj_pos_x;
	double	obj_pos_y;
	double	obj_pos_z;
}Data3D;

//////////////////////////
// ‘ÎÛ•¨‘Ìî•ñ
//////////////////////////
typedef struct{
	DataCam		cam;
	Data3D		dim3;
}DATA;

int storeInDATA(DATA *data, double *input);

#endif //_INC_DATA

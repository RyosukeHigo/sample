#ifndef _INC_DATADEF
#define _INC_DATADEF

/////////////////////////////////////////////////
// データストア
// 最終行定義 DATA_*_NUM は要素数
/////////////////////////////////////////////////

// Image
#define IMG1_VISIBLE	0
#define IMG2_VISIBLE	1
#define IMG1_POS_X		2
#define IMG1_POS_Y		3
#define IMG2_POS_X		4
#define IMG2_POS_Y		5
#define DATA_IMG_NUM	6

// Object
typedef double *Object;
#define OBJ_VISIBLE		0
#define OBJ_POS_X		1
#define OBJ_POS_Y		2
#define OBJ_POS_Z		3
#define DATA_OBJ_NUM	4

/*
// データストアに対応する
typedef enum {
	AVS1_X,
	AVS1_Y,
	AVS2_X,
	AVS2_Y,
	AVS1_VISIBLE,
	AVS2_VISIBLE,
	IMG_STATE_NUM		// 要素の個数
} ImgState;

typedef enum {
	AVS1_VISIBLE,
	AVS2_VISIBLE,
	OBJ_POS_X,
	OBJ_POS_Y,
	OBJ_POS_Z
} ObjState;
*/

#endif

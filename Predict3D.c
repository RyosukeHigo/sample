#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <myMatrix.h>
#include <Kalman.h>
#include "Predict3D.h"

//////////////////////////////////////////////////////////////////
static dMatrix  Hmat, Pmat, Umat;
static dVector  avec, yvec, rvec;
static Vector3D obj_pos_init;
static int      count_init;

void predict3D_init(void)
{
    int i, j;
    int N, IP;

    N  = 6; //パラメータ数
    IP = 3; //データ数

    //行列の定義
    //  y = H a
    Hmat   = dmatZeros(IP, N);  // 
    avec   = dvecZeros(N);     // パラメータ
    yvec   = dvecZeros(IP);     // 現在データ
    Pmat   = dmatZeros(N, N);  // 共分散逆行列
    Umat   = dmatZeros(N, N);  // U行列
    rvec   = dvecZeros(IP);    //観測誤差の分散

    // 初期値
    for(i=0; i<N;  i++) Pmat.el[i][i] = 10000.0;
    for(i=0; i<IP; i++) rvec.el[i] = 1.0;    
    // UD 分解は最初に一回
    mat2UD(Umat, Pmat);
    for(i=0; i<N; i++)	for(j=0; j<N; j++)   Pmat.el[i][j] = Umat.el[i][j];
    return;
}

void predict3D_start(Vector3D obj_pos, int count)
{
    int crd, i, j;

    // 初期値の設定
    count_init = count;
    for(crd=0; crd<3; crd++) obj_pos_init[crd] = obj_pos[crd];
    for(i=0; i<avec.n; i++) avec.el[i] = 0.0;
    // 初期値
    for(i=0; i<Umat.row; i++)	for(j=0; j<Umat.col; j++)  Umat.el[i][j] =  Pmat.el[i][j];
    return;
}

void predict3D(Vector3D obj_pos_predict, int count_bias, Vector3D obj_pos, int count, double interval)
{
    int crd;
    double time, timef;

    time = (count - count_init)*interval;
    // 値の設定
    Hmat.el[0][0] = Hmat.el[1][1] = Hmat.el[2][2] = time;;
    Hmat.el[0][3] = Hmat.el[1][4] = Hmat.el[2][5] = time*time;
    for(crd=0; crd<3; crd++) yvec.el[crd] = obj_pos[crd]-obj_pos_init[crd];
    // 観測値の更新
    kalmanObservUpdateUD(avec, Umat, yvec, Hmat, rvec);
    //軌道予測
    timef = (count_bias + count - count_init)*interval;

    obj_pos_predict[0] = avec.el[3]*timef*timef + avec.el[0]*timef + obj_pos_init[0];
    obj_pos_predict[1] = avec.el[4]*timef*timef + avec.el[1]*timef + obj_pos_init[1];
    obj_pos_predict[2] = avec.el[5]*timef*timef + avec.el[2]*timef + obj_pos_init[2];
    //printf("%f %f %f %f %f %f %f\n", 
    //time, avec.el[0], avec.el[1], avec.el[2], avec.el[3], avec.el[4], avec.el[5]);
}


void predict3D_calc(Vector3D obj_pos_predict,
		    int      count_bias, 
		    int      count, 
		    double    interval)
{
    static int counter = 0;
    int i;
    double timef;

    timef = (count_bias + count - count_init)*interval;
    obj_pos_predict[0] = avec.el[3]*timef*timef + avec.el[0]*timef + obj_pos_init[0];
    obj_pos_predict[1] = avec.el[4]*timef*timef + avec.el[1]*timef + obj_pos_init[1];
    obj_pos_predict[2] = avec.el[5]*timef*timef + avec.el[2]*timef + obj_pos_init[2];
    counter ++;
}

void predict_trajectory(Vector3D obj_pos, double a[6], int count, double interval)
{
    int crd,i;
    double time, timef;

    time = (count - count_init)*interval;
    // 値の設定
    Hmat.el[0][0] = Hmat.el[1][1] = Hmat.el[2][2] = time;;
    Hmat.el[0][3] = Hmat.el[1][4] = Hmat.el[2][5] = time*time;
    for(crd=0; crd<3; crd++) yvec.el[crd] = obj_pos[crd]-obj_pos_init[crd];
    // 観測値の更新
    kalmanObservUpdateUD(avec, Umat, yvec, Hmat, rvec);
    // 軌道パラメータ取得
    for(i=0; i<6; i++) a[i] = avec.el[i];
}

void predict3D_exit(void)
{
    // 解放
    dvecFree(yvec);
    dvecFree(avec);
    dmatFree(Hmat);
    dmatFree(Umat);
    dvecFree(rvec);
    dmatFree(Pmat);
}

void predict3D_get(double a[6])
{
    int i;
    for(i=0; i<6; i++) a[i] = avec.el[i];
}

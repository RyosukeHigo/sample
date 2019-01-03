#include <stdio.h>
#include <math.h>
void Yturn_Rolling();
int main()
{
    printf("Hello, World!\n");
    Yturn_Rolling();
    return 0;
}

void Yturn_Rolling()
{
	double d;						//ロボット座標系原点からみた指根関節の座標
	double a_1;						//指根元関節角
	double a_2;						//指先関節角
	double r = 0.045 - 0.0365;		//指先半径
	double cube_angle = 0.0;				//弧度法で表したキューブの角度
	double f;						//指先の接触点の角度
	double L_1 = 0.062;				//指先根元リンク
	double f2;						// = a_1 + a_2 + (90 - angle) * PI / 180 - PI / 2;
	double L_2 = 0.0365;			//指先リンク
	double x_f;						//指先のx座標
	double y_f;						//指先のy座標
	double omega = M_PI / 3;			//目標とするルービックキューブの角速度
	double O_x = 0.1503 - 0.045; //ロボット座標系から見たルービックキューブの重心位置のx座標
	double O_y = 0.0;			//ロボット座標系から見たルービックキューブの重心位置のy座標
	double u_1;						//接触点座標系からみたルービックキューブの重心の位置
	double u_3 = 0.0555 / 2;		//接触点座標系からみたルービックキューブの重心の位置
	double tmp;
	double tmp2;
	double A[3][3]; //逆行列
	double S12f;
	double C12f;
	double S12;
	double C12;
	double da_1; //関節角の速度
	double da_2; //関節角の速度
	double da_5; //関節角の速度
	double da_6; //関節角の速度
	double angle = 3.1;
	if (angle > 45.0)
	{
		cube_angle = (90.0 - angle) * M_PI / 180.0;
	}
	else
	{
		cube_angle = -angle * M_PI / 180.0;
	}

	//キューブの回転方向によって軌道を変える
	//左指
	d = 0.06;

	a_1 = 0.0048;//-hand.var.jnt_ang[HAND_M5]; //指根元関節角
	a_2 = 0.8211;//-0.8168;//-0.882;//-hand.var.jnt_ang[HAND_M6];
	x_f = L_2 * cos(a_1 + a_2) + L_1 * cos(a_1);	  //指先のx座標
	y_f = -L_2 * sin(a_1 + a_2) - L_1 * sin(a_1) + d; //指先のy座標
	f = a_1 + a_2 + cube_angle - M_PI / 2;
	//u_1 = -(tan(cube_angle) * x_f - y_f - tan(cube_angle) * O_x + O_y) / sqrt(tan(cube_angle) * tan(cube_angle) + 1);
	u_1 = (tan(cube_angle) * (-y_f) - x_f + O_x + tan(cube_angle) * O_y) / sqrt(tan(cube_angle) * tan(cube_angle) + 1);
	tmp = sin(a_2 - f) * u_3 - cos(a_2 - f) * u_1 - L_2 * sin(a_2);
	tmp2 = sin(a_2 - f) * u_3 + cos(a_2 - f) * u_1 - L_2 * sin(a_2);
	S12f = sin(a_1 + a_2 - f);
	C12f = cos(a_1 + a_2 - f);
	S12 = sin(a_1 + a_2);
	C12 = cos(a_1 + a_2);
	if (omega < 0)
	{
		u_1 = -u_1;
	}
    A[0][0] = (u_1 * sin(a_2 + a_1 - f) + u_3 * cos(a_2 + a_1 - f) + r * cos(a_2 + a_1 - f) - u_1 * sin(a_2 + a_1 - 2 * f) - u_3 * cos(a_2 + a_1 - 2 * f) + L_2 * cos(a_2 + a_1)) / (L_1 * (u_3 * sin(a_2 - f) + r * sin(a_2 - f) - u_1 * cos(a_2 - f) - u_3 * sin(a_2 - 2 * f) + u_1 * cos(a_2 - 2 * f) + L_2 * sin(a_2)));
	A[0][1] = -(u_3 * sin(a_2 + a_1 - f) + r * sin(a_2 + a_1 - f) - u_1 * cos(a_2 + a_1 - f) - u_3 * sin(a_2 + a_1 - 2 * f) + u_1 * cos(a_2 + a_1 - 2 * f) + L_2 * sin(a_2 + a_1)) / (L_1 * (u_3 * sin(a_2 - f) + r * sin(a_2 - f) - u_1 * cos(a_2 - f) - u_3 * sin(a_2 - 2 * f) + u_1 * cos(a_2 - 2 * f) + L_2 * sin(a_2)));
	A[0][2] = -(d * u_1 * sin(a_2 + a_1 - f) + d * u_3 * cos(a_2 + a_1 - f) + d * r * cos(a_2 + a_1 - f) - d * u_1 * sin(a_2 + a_1 - 2 * f) - d * u_3 * cos(a_2 + a_1 - 2 * f) + L_2 * d * cos(a_2 + a_1) + L_1 * u_3 * sin(a_2 - f) + L_1 * r * sin(a_2 - f) - L_1 * u_1 * cos(a_2 - f) - L_1 * u_3 * sin(a_2 - 2 * f) + L_1 * u_1 * cos(a_2 - 2 * f) + L_1 * L_2 * sin(a_2)) / (L_1 * (u_3 * sin(a_2 - f) + r * sin(a_2 - f) - u_1 * cos(a_2 - f) - u_3 * sin(a_2 - 2 * f) + u_1 * cos(a_2 - 2 * f) + L_2 * sin(a_2)));

	A[1][0] = -(u_1 * sin(a_2 + a_1 - f) + u_3 * cos(a_2 + a_1 - f) + r * cos(a_2 + a_1 - f) - u_1 * sin(a_2 + a_1 - 2 * f) - u_3 * cos(a_2 + a_1 - 2 * f) + L_2 * cos(a_2 + a_1) + L_1 * cos(a_1)) / (L_1 * (u_3 * sin(a_2 - f) + r * sin(a_2 - f) - u_1 * cos(a_2 - f) - u_3 * sin(a_2 - 2 * f) + u_1 * cos(a_2 - 2 * f) + L_2 * sin(a_2)));
	A[1][1] = (u_3 * sin(a_2 + a_1 - f) + r * sin(a_2 + a_1 - f) - u_1 * cos(a_2 + a_1 - f) - u_3 * sin(a_2 + a_1 - 2 * f) + u_1 * cos(a_2 + a_1 - 2 * f) + L_2 * sin(a_2 + a_1) + L_1 * sin(a_1)) / (L_1 * (u_3 * sin(a_2 - f) + r * sin(a_2 - f) - u_1 * cos(a_2 - f) - u_3 * sin(a_2 - 2 * f) + u_1 * cos(a_2 - 2 * f) + L_2 * sin(a_2)));
	A[1][2] = (d * (u_1 * sin(a_2 + a_1 - f) + u_3 * cos(a_2 + a_1 - f) + r * cos(a_2 + a_1 - f) - u_1 * sin(a_2 + a_1 - 2 * f) - u_3 * cos(a_2 + a_1 - 2 * f) + L_2 * cos(a_2 + a_1) + L_1 * cos(a_1))) / (L_1 * (u_3 * sin(a_2 - f) + r * sin(a_2 - f) - u_1 * cos(a_2 - f) - u_3 * sin(a_2 - 2 * f) + u_1 * cos(a_2 - 2 * f) + L_2 * sin(a_2)));

	da_1 = A[0][0] * (O_y * omega) + A[0][1] * (-O_x * omega) + A[0][2] * omega;
	da_2 = A[1][0] * (O_y * omega) + A[1][1] * (-O_x * omega) + A[1][2] * omega;
	
	printf("%f \n", u_1);
	da_5 = A[0][0] * (O_y * omega) + A[0][1] * (-O_x * omega) + A[0][2] * omega;
	da_6 = A[1][0] * (O_y * omega) + A[1][1] * (-O_x * omega) + A[1][2] * omega;

    printf("d = %f \n", d);
	printf("A[0][0] = %f, A[0][1] = %f, A[0][2] = %f  \n",A[0][0], A[0][1], A[0][2]);
	printf("A[1][0] = %f, A[1][1] = %f, A[1][2] = %f  \n",A[1][0], A[1][1], A[1][2]);
	printf("%f \n",da_1);
	printf("%f \n",da_2);
}
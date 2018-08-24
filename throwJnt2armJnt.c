#include <math.h>
#include <common_definition.h>

/////////////////////////////////////////////////////
// �����X�E�B���O���f���֐�(3DOF)���o���b�g�֐�(4DOF)�ɕϊ�
/////////////////////////////////////////////////////
int throwModelJnt2armJnt(double *qa, double *dqa, double *ddqa,
					double *qt, double *dqt, double *ddqt)
{
	int	jnt;
	double alpha, dalpha, ddalpha;
	double dqa3_up, dqa3_down, dqa4_1, dqa4_2;
	double ddqa3_up, ddqa3_down, ddqa4_1, ddqa4_2;

	alpha = qt[2];	dalpha = dqt[2];	ddalpha = ddqt[2];

	for(jnt=0;jnt<2;jnt++){
		qa[jnt] = qt[jnt];
		dqa[jnt] = dqt[jnt];
		ddqa[jnt] = ddqt[jnt];
	}

	// �p�x
	qa[2] = atan2(tan(alpha), cos(qa[1]));
	qa[3] = atan2(1, tan(qa[1])*cos(qa[2]));

	// �p���x
	dqa3_up = cos(qa[1])*dalpha+sin(alpha)*cos(alpha)*sin(qa[1])*dqa[1];
	dqa3_down = cos(qa[1])*cos(qa[1])*cos(alpha)*cos(alpha)+sin(alpha)*sin(alpha);
	dqa[2] = dqa3_up/dqa3_down;
	dqa4_1 = 1/(sin(qa[1])*sin(qa[1])*cos(qa[2])*cos(qa[2])+cos(qa[1])*cos(qa[1]));
	dqa4_2 = cos(qa[2])*dqa[1]-sin(qa[1])*cos(qa[1])*sin(qa[2])*dqa[2];
	dqa[3] = -dqa4_1*dqa4_2;

	// �p�����x
	ddqa3_up = ( cos(qa[1])*ddalpha + sin(alpha)*cos(alpha)*sin(qa[1])*ddqa[1]
		- sin(qa[1])*dalpha*dqa[1] + cos(alpha)*cos(alpha)*sin(qa[1])*dqa[1]*dalpha
		- sin(alpha)*sin(alpha)*sin(qa[1])*dqa[1]*dalpha + sin(alpha)*cos(alpha)*cos(qa[1])*dqa[1]*dqa[1] ) * dqa3_down
		- dqa3_up * ( - 2*cos(qa[1])*sin(qa[1])*cos(alpha)*cos(alpha)*dqa[1] - 2*cos(qa[1])*cos(qa[1])*cos(alpha)*sin(alpha)*dalpha
		+ 2*sin(alpha)*cos(alpha)*dalpha );
	ddqa3_down = dqa3_down*dqa3_down;
	ddqa[2] = ddqa3_up/ddqa3_down;
	ddqa4_1 = -dqa4_1*dqa4_1 * ( 2*sin(qa[1])*cos(qa[1])*cos(qa[2])*cos(qa[2])*dqa[1] - 2*sin(qa[1])*sin(qa[1])*cos(qa[2])*sin(qa[2])*dqa[2]
		- 2*cos(qa[1])*sin(qa[1])*dqa[1] );
	ddqa4_2 = cos(qa[2])*ddqa[1]-sin(qa[1])*cos(qa[1])*sin(qa[2])*ddqa[2]
		- sin(qa[2])*dqa[1]*dqa[2] - cos(qa[1])*cos(qa[1])*sin(qa[2])*dqa[2]*dqa[1]
		+ sin(qa[1])*sin(qa[1])*sin(qa[2])*dqa[2]*dqa[1] - sin(qa[1])*cos(qa[1])*cos(qa[2])*dqa[2]*dqa[2];
	ddqa[3] = -(ddqa4_1*dqa4_2+dqa4_1*ddqa4_2);

	return 0;
}

/////////////////////////////////////////////////////
// �o�b�e�B���O�X�E�B���O���f���֐�(4DOF)���o���b�g�֐�(4DOF)�ɕϊ�
/////////////////////////////////////////////////////
int battingModelJnt2armJnt(double *qa, double *dqa, double *ddqa,
					double *qt, double *dqt, double *ddqt)
{
	int	jnt;
	double x, y, z;

	x = sin(qt[1]+qt[3])*cos(qt[2]);
	y = sin(qt[1]+qt[3])*sin(qt[2]);
	z = cos(qt[1]+qt[3]);


	for(jnt=0;jnt<2;jnt++){
		qa[jnt] = qt[jnt];	dqa[jnt] = dqt[jnt];	ddqa[jnt] = ddqt[jnt];
	}

	// �p�x
	qa[2] = atan2(y, cos(qa[1])*x-sin(qa[1])*z);
//	if(y < 0)	qa[2] += PI;

//	if(fabs(y)>0.001)	qa[3] = atan2(y, sin(qa[2])*(sin(qa[1])*x+cos(qa[1])*z));
//	else	qa[3] = acos(sin(qa[1])*x+cos(qa[1])*z);

	//if(z < cos(qt[1]))	qa[3] = acos(sin(qa[1])*x+cos(qa[1])*z);
	//else	qa[3] = -acos(sin(qa[1])*x+cos(qa[1])*z);
	qa[3] = acos(sin(qa[1])*x + cos(qa[1])*z);

	//�p���x
	// 2���͌Œ�i�p���x0�j
	dqa[0] = dqt[0];
	dqa[1] = 0;
	dqa[2] = -(sin(qt[1])*sin(qt[2])*cos(qt[1] + qt[3])*cos(qt[1] + qt[3]) * dqt[3] - cos(qt[1])*sin(qt[2])*sin(qt[2]) * sin(qt[1] + qt[3]) * sin(qt[1] + qt[3]) * dqt[2] - cos(qt[1])*cos(qt[2]) *cos(qt[2]) * sin(qt[1] + qt[3])* sin(qt[1] + qt[3]) * dqt[2] + sin(qt[1])*sin(qt[2])*sin(qt[1] + qt[3])*sin(qt[1] + qt[3]) * dqt[3] + cos(qt[2])*sin(qt[1])*cos(qt[1] + qt[3])*sin(qt[1] + qt[3])*dqt[2]) / (sin(qt[1]) *sin(qt[1]) * cos(qt[1] + qt[3])* cos(qt[1] + qt[3]) + sin(qt[2]) *sin(qt[2]) * sin(qt[1] + qt[3]) * sin(qt[1] + qt[3]) + cos(qt[1]) *cos(qt[1]) * cos(qt[2]) * cos(qt[2]) * sin(qt[1] + qt[3]) * sin(qt[1] + qt[3]) - 2 * cos(qt[1])*cos(qt[2])*sin(qt[1])*cos(qt[1] + qt[3])*sin(qt[1] + qt[3]));
	if (z < cos(qt[1])){
		dqa[3] = (cos(qt[1])*sin(qt[1] + qt[3])*dqt[3] - cos(qt[2])*sin(qt[1])*cos(qt[1] + qt[3])*dqt[3] + sin(qt[1])*sin(qt[2])*sin(qt[1] + qt[3])*dqt[2]) / sqrt(-cos(qt[1])*cos(qt[1]) * cos(qt[1] + qt[3]) * cos(qt[1] + qt[3]) - 2 * cos(qt[1])*cos(qt[2])*sin(qt[1])*cos(qt[1] + qt[3])*sin(qt[1] + qt[3]) - cos(qt[2])*cos(qt[2]) * sin(qt[1])* sin(qt[1]) * sin(qt[1] + qt[3])* sin(qt[1] + qt[3]) + 1);
	}
	else{
		dqa[3] = -(cos(qt[1])*sin(qt[1] + qt[3])*dqt[3] - cos(qt[2])*sin(qt[1])*cos(qt[1] + qt[3])*dqt[3] + sin(qt[1])*sin(qt[2])*sin(qt[1] + qt[3])*dqt[2]) / sqrt(-cos(qt[1])*cos(qt[1]) * cos(qt[1] + qt[3]) * cos(qt[1] + qt[3]) - 2 * cos(qt[1])*cos(qt[2])*sin(qt[1])*cos(qt[1] + qt[3])*sin(qt[1] + qt[3]) - cos(qt[2])*cos(qt[2]) * sin(qt[1])* sin(qt[1]) * sin(qt[1] + qt[3])* sin(qt[1] + qt[3]) + 1);
	}

	return 0;
}

/////////////////////////////////////////////////////
// �o���b�g�֐�(4DOF)���o�b�e�B���O�X�E�B���O���f���֐�(4DOF)�ɕϊ�
/////////////////////////////////////////////////////
int armJnt2battingModelJnt(double *qt, double *dqt, double *ddqt,
					double *qa, double *dqa, double *ddqa)
{
	int	jnt;
	double x, y, z;

	x = sin(qa[1])*cos(qa[3])+cos(qa[1])*cos(qa[2])*sin(qa[3]);
	y = sin(qa[2])*sin(qa[3]);
	z = cos(qa[1])*cos(qa[3])-sin(qa[1])*cos(qa[2])*sin(qa[3]);


	for(jnt=0;jnt<2;jnt++){
		qt[jnt] = qa[jnt];	dqt[jnt] = dqa[jnt];	ddqt[jnt] = ddqa[jnt];
	}

	// �p�x
	qt[2] = atan2(y, x);
	if(cos(qt[1]) > z)	qt[3] = atan2(sqrt(x*x+y*y), z)-qt[1];
	else	qt[3] = atan2(-sqrt(x*x+y*y), z)-qt[1];

	//�p���x
	// 2���͌Œ�i�p���x0�j
	dqt[0] = dqa[0];
	dqt[1] = 0;
	dqt[2] = (cos(qa[1])*dqa[2] - cos(qa[1])*cos(qa[3])*cos(qa[3]) * dqa[2] + sin(qa[1])*sin(qa[2])*dqa[3] + cos(qa[2])*cos(qa[3])*sin(qa[1])*sin(qa[3])*dqa[2]) / (cos(qa[3])*cos(qa[3]) * sin(qa[1]) * sin(qa[1]) + sin(qa[2]) * sin(qa[2])* sin(qa[3])* sin(qa[3]) + cos(qa[1]) *cos(qa[1]) * cos(qa[2]) * cos(qa[2]) * sin(qa[3]) * sin(qa[3]) + 2 * cos(qa[1])*cos(qa[2])*cos(qa[3])*sin(qa[1])*sin(qa[3]));
	if (z < cos(qt[1])){
		dqt[3] = (cos(qa[1])*sin(qa[3])*dqa[3] + cos(qa[2])*cos(qa[3])*sin(qa[1])*dqa[3] - sin(qa[1])*sin(qa[2])*sin(qa[3])*dqa[2]) / sqrt(-cos(qa[1])*cos(qa[1]) * cos(qa[2])* cos(qa[2]) * cos(qa[3])* cos(qa[3]) + cos(qa[1])*cos(qa[1]) * cos(qa[2]) * cos(qa[2]) - cos(qa[1]) *cos(qa[1]) * cos(qa[3])* cos(qa[3]) + 2 * sin(qa[1])*sin(qa[3])*cos(qa[1])*cos(qa[2])*cos(qa[3]) + cos(qa[2]) *cos(qa[2]) * cos(qa[3])*cos(qa[3]) - cos(qa[2]) *cos(qa[2]) + 1);
	}
	else{
		dqt[3] = -(cos(qa[1])*sin(qa[3])*dqa[3] + cos(qa[2])*cos(qa[3])*sin(qa[1])*dqa[3] - sin(qa[1])*sin(qa[2])*sin(qa[3])*dqa[2]) / sqrt(-cos(qa[1])*cos(qa[1]) * cos(qa[2])* cos(qa[2]) * cos(qa[3])* cos(qa[3]) + cos(qa[1])*cos(qa[1]) * cos(qa[2]) * cos(qa[2]) - cos(qa[1]) *cos(qa[1]) * cos(qa[3])* cos(qa[3]) + 2 * sin(qa[1])*sin(qa[3])*cos(qa[1])*cos(qa[2])*cos(qa[3]) + cos(qa[2]) *cos(qa[2]) * cos(qa[3])*cos(qa[3]) - cos(qa[2]) *cos(qa[2]) + 1);
	}

	return 0;
}

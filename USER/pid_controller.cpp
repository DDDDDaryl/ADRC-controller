#include "pid_controller.h"
#include "usart.h"
#include <cmath>

PID::PID() {
    IncPIDInit();
}

void PID::IncPIDInit()
{
	SetPoint	= 1;		//�趨ֵ
	BitMove	= 0;		//���ؽ������

	LastError = 0;		//ǰ2�����ֵ
	PrevError = 0;		//ǰ1�����ֵ

	Proportion = 100;		//����
	Integral	 = 1;		//����
	Derivative = 0;		//΢��

	iError = 0;			//��ǰ���
	iIncpid=0;			//�������

	Uk = 0;				//�������ֵ
	Is_not_negative = 0;
}

float PID::IncPIDCalc(float NextPoint, float sp)
{
    //printf("Kp = %f\r\n", Proportion);
    SetPoint = sp;
	//��ǰ���
	iError = SetPoint - NextPoint;
	//�������
	iIncpid= Proportion * (iError - LastError)
				+ Integral * iError
				+ Derivative * (iError - 2 * LastError + PrevError );
    
	//�洢�������´μ���
	PrevError = LastError;
	LastError = iError;

	Uk += iIncpid;
    Uk = Uk >= 0 ? fmin(Uk, MAXOUT) : fmax(Uk, -MAXOUT);
    
    return Uk;

	//���ֵ�޷�
//	if (Uk >= MAXOUT)
//	{
//		Is_not_negative = 1;
//		return MAXOUT;
//	}
//	else if(Uk <= 0 && Uk >= (-1)*MAXOUT)
//	{
//		Is_not_negative = 0;
//		return fabs( Uk );
//	}
//	else if(Uk < (-1) * MAXOUT)
//	{
//		Is_not_negative = 0;
//		return MAXOUT;
//	}
//	else	
//	{
//		Is_not_negative = 1;
//		return Uk;
//	}
	
}

void PID::pid_init(float kp, float ki, float kd, float ref) {
	SetPoint = ref;
	
	Proportion = kp;
	Integral = ki;
	Derivative = kd;
}
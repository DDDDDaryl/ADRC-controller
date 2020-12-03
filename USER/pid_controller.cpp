#include "pid_controller.h"
#include "usart.h"
#include <cmath>

PID::PID() {
    IncPIDInit();
}

void PID::IncPIDInit()
{
	SetPoint	= 1;		//设定值
	BitMove	= 0;		//返回结果比例

	LastError = 0;		//前2次误差值
	PrevError = 0;		//前1次误差值

	Proportion = 100;		//比例
	Integral	 = 1;		//积分
	Derivative = 0;		//微分

	iError = 0;			//当前误差
	iIncpid=0;			//增量误差

	Uk = 0;				//输出返回值
	Is_not_negative = 0;
}

float PID::IncPIDCalc(float NextPoint, float sp)
{
    //printf("Kp = %f\r\n", Proportion);
    SetPoint = sp;
	//当前误差
	iError = SetPoint - NextPoint;
	//增量误差
	iIncpid= Proportion * (iError - LastError)
				+ Integral * iError
				+ Derivative * (iError - 2 * LastError + PrevError );
    
	//存储误差，用于下次计算
	PrevError = LastError;
	LastError = iError;

	Uk += iIncpid;
    Uk = Uk >= 0 ? fmin(Uk, MAXOUT) : fmax(Uk, -MAXOUT);
    
    return Uk;

	//输出值限幅
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
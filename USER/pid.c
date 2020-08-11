#define _PID_C
#include "pid.h"
#include <math.h>

#define MAXOUT 3300				//输出最大值

void IncPIDInit(void)
{
	sptr			= &sPID;
	sptr->SetPoint	= 0;		//设定值
	sptr->BitMove	= 0;		//返回结果比例

	sptr->LastError = 0;		//前2次误差值
	sptr->PrevError = 0;		//前1次误差值

	sptr->Proportion = 25;		//比例
	sptr->Integral	 = 0;		//积分
	sptr->Derivative = 0;		//微分

	sptr->iError = 0;			//当前误差
	sptr->iIncpid=0;			//增量误差

	sptr->Uk = 0;				//输出返回值
	sptr->Is_not_negative = 0;
}

float IncPIDCalc(float NextPoint)
{
	//当前误差
	sptr->iError = sptr->SetPoint - NextPoint;
	//增量误差
	sptr->iIncpid= sptr->Proportion * (sptr->iError - sptr->LastError)
				+ sptr->Integral * sptr->iError
				+ sptr->Derivative * (sptr->iError - 2*sptr->LastError + sptr->PrevError );
	//存储误差，用于下次计算
	sptr->PrevError = sptr->LastError;
	sptr->LastError = sptr->iError;

	sptr->Uk += sptr->iIncpid;

	//输出值限幅
	if (sptr->Uk >= MAXOUT)
	{
		sptr->Is_not_negative = 1;
		return(MAXOUT);
	}
	else if(sptr->Uk <= 0 && sptr->Uk >= (-1)*MAXOUT)
	{
		sptr->Is_not_negative = 0;
		return(fabs(sptr->Uk));
	}
	else if(sptr->Uk < (-1)*MAXOUT)
	{
		sptr->Is_not_negative = 0;
		return(MAXOUT);
	}
	else	
	{
		sptr->Is_not_negative = 1;
		return(sptr->Uk);
	}
	
	
}
#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

/*------------------------增量PID说明-------------------------*/
//增量式PID的输出为delta(uk)即控制量的增量
		//delta(uk)=u(k)-u(k-1)=kp*delta(ek)+ki*delta(ek)+kd*[delta(ek)-delta(ek-1)]
		//可化简为delta(uk)=Ae(k)-Be(k-1)+Ce(k-2)
		//其中A=kp*(1+T/Ti+Td/T)
		//		B=kp*(1+2Td/T)
		//		C=kp*kd/T
		//式中，T为采样周期，Ti为积分时间，Td为微分时间
		//PID中，kp=kp,ki=kp/Ti,kd=kp*Td
/*------------------------------------------------------------*/

class PID {
	private:
		float SetPoint;
	
		unsigned char BitMove;
	
		float Proportion;
		float Integral;
		float Derivative;
	
		float iError;
		float iIncpid;
	
		float LastError;
		float PrevError;
	
		float Uk;
		int Is_not_negative;
	
		const float MAXOUT = 3.3;
	
	public:
        PID();    
    
		void IncPIDInit();
	
		float IncPIDCalc(float NextPoint, float sp);
	
		void pid_init(float kp, float ki, float kd, float ref);
};

#endif
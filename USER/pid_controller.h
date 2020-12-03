#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

/*------------------------����PID˵��-------------------------*/
//����ʽPID�����Ϊdelta(uk)��������������
		//delta(uk)=u(k)-u(k-1)=kp*delta(ek)+ki*delta(ek)+kd*[delta(ek)-delta(ek-1)]
		//�ɻ���Ϊdelta(uk)=Ae(k)-Be(k-1)+Ce(k-2)
		//����A=kp*(1+T/Ti+Td/T)
		//		B=kp*(1+2Td/T)
		//		C=kp*kd/T
		//ʽ�У�TΪ�������ڣ�TiΪ����ʱ�䣬TdΪ΢��ʱ��
		//PID�У�kp=kp,ki=kp/Ti,kd=kp*Td
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
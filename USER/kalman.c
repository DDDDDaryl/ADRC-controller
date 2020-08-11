#include "kalman.h"

KFP KFP_height={0.02,0,0,0,0.001,0.543};

/**
*�������˲���
*@param KFP *kfp �������ṹ�����
*   float input ��Ҫ�˲��Ĳ����Ĳ���ֵ�����������Ĳɼ�ֵ��
*@return �˲���Ĳ���������ֵ��
*/
float kalmanFilter(KFP *kfp,float input)
{
	 //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
	 kfp->Now_P = kfp->LastP + kfp->Q;
	 //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
	 kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
	 //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
	 kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
	 //����Э�����: ���ε�ϵͳЭ����� kfp->LastP ����һ������׼����
	 kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
	 return kfp->out;
}

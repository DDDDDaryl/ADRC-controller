/*coding in GBK*/

#include <chrono>
#include "protocol.h"
#include "objects.h"
#include "usart.h"
#include "sys.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "adc.h"
#include "dac.h"

using namespace std;
using namespace chrono;

extern u16 USART_RX_STA;
extern u8  Tim2Flag;

int main(){
/*----------------------��ʵ����-----------------------*/
    control_system sys;

    C__init();
    C__ID_hash_table_init();

    ESO ESO_3rd(3);
    ESO_3rd.LADRC_based_current_DESO_init();

    controller ctrl;
    ctrl.Parameter_init(ctrl);
/*----------------------�������ʼ��-----------------------*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		//��ʱ��ʼ��
	uart_init(115200);	//���ڳ�ʼ��������Ϊ115200
	USART2_Init(115200);
	MCU1UART_DMA_cinfig();                         //����DMAͨ�ų�ʼ��

	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();       //��ʼ���밴�����ӵ�Ӳ���ӿ�

	Adc_Init(); 				//adc��ʼ��
	Dac1_Init();		 		//DACͨ��1��ʼ��
	Dac2_Init();		 		//DACͨ��1��ʼ��
	DAC_TIM_Config();

	DAC_SetChannel1Data(DAC_Align_12b_R,0);//��ʼֵΪ0
	DAC_SetChannel2Data(DAC_Align_12b_R,0);//��ʼֵΪ0

    while(true){//֮��ĳ�1
    /*---------------------------���ڽ������ݽ���----------------------------*/
        if(C__get_PC_parse_flag() == true){
            USART_RX_STA = 0;
            C__set_PC_parse_flag(false);
            C__parse_PC_msg();
        }
        while(control_system::get_system_state() ){//ϵͳ������
            /*---------------------------���ڽ������ݽ���---------------------------*/
            if(C__get_PC_parse_flag() == true){                
                printf("PC Message received.\r\n");
                USART_RX_STA = 0;
                C__set_PC_parse_flag(false);
                C__parse_PC_msg();
                
                continue;
            }

            if(C__get_IC_parse_flag()){//ֱ�Ӹ���reference
                printf("IC Message received.\r\n");
                USART_RX_STA = 0;
                C__set_IC_parse_flag(false);
                printf("reference = %#1.0f\r\n", C__parse_IC_msg());
                //C__parse_IC_msg();
            }
            if(Tim2Flag==1)
                {
                switch (control_system::get_Is_close_loop()){
                    case closed_loop:{

                        switch(control_system::get_controller_type()){
                            case LADRC:{
    //                            /*-------------------------------���Բ���---------------------------*/
    //                            auto start = system_clock::now();
    //                            /*-----------------------------�������ײ�---------------------------*/
                                float u = control_system::update_control_signal_V(ctrl.Iterate(ESO::get_output(), control_system::get_reference())[0][0]);
                                ESO_3rd.Iterate();//�õ���ǰ���
                                //printf("Control Signal = %#1.1f\r\n", u);
                                /*-------------------------------���Բ���---------------------------*/
    //                            auto end   = system_clock::now();
    //                            auto duration = duration_cast<microseconds>(end - start);
    //                            printf("%f us spent.\r\n", double(duration.count()));
                                /*-----------------------------�������ײ�---------------------------*/
                                break;
                            }
                            case PID:{
                                break;
                            }
                        }//switch(sys.get_controller_type())
                        break;
                    }//case closed_loop:
                    case open_loop:{

                        switch(control_system::get_open_loop_input_type()){
                            case sine:{
                                break;
                            }
                            case step:{
                                break;
                            }
                        }//switch(sys.get_open_loop_input_type())
                        break;
                    }//case open_loop:
                }//switch (sys.get_Is_close_loop())
            }
        }//while(sys.get_system_state() )
        /*------------------���³�ʼ��������----------------*/
        //printf("System running stopped.");
    }//while(1)

}





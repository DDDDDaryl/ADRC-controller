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

extern u8 debug_fl;

int main(){
/*----------------------��ʵ����-----------------------*/
    control_system sys;

    C__init();
    C__ID_hash_table_init();

    ESO ESO_3rd(3);
    ESO_3rd.LADRC_based_current_DESO_init();

    controller ctrl;
    ctrl.Parameter_init();
    
/*----------------------�������ʼ��-----------------------*/

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		//��ʱ��ʼ��
	uart_init(115200);	//���ڳ�ʼ��������Ϊ115200
	USART2_Init(115200);
    USART3_Init(115200);
	MCU1UART_DMA_cinfig();                         //����DMAͨ�ų�ʼ��

	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();       //��ʼ���밴�����ӵ�Ӳ���ӿ�


	Adc_Init(); 				//adc��ʼ��
	Dac1_Init();		 		//DACͨ��1��ʼ��
	Dac2_Init();		 		//DACͨ��1��ʼ��
	DAC_TIM_Config();

	DAC_SetChannel1Data(DAC_Align_12b_R,0);//��ʼֵΪ0
	DAC_SetChannel2Data(DAC_Align_12b_R,0);//��ʼֵΪ0
	printf("Initializing...\r\n");
	
	unsigned long long cnt = 0;
	unsigned long long heart_beat_cnt = 0;
	
	//set_output(3.3f);
	
    int sample_period_in_ms = 1000 / control_system::get_Sample_Rate_Hz();
    while(true) {//֮��ĳ�1
		
        
    /*---------------------------���ڽ������ݽ���----------------------------*/
        if(C__get_PC_parse_flag() == true){
            USART_RX_STA = 0;
            C__set_PC_parse_flag(false);
            C__parse_PC_msg();
            /*���Ӹ��²�������*/
            sample_period_in_ms = 1000 / control_system::get_Sample_Rate_Hz();
        }
        
        /*ÿ��1s������������������λ���̵߳ȴ���*/
		if (!control_system::get_system_state()) {
			if(Tim2Flag==1) {
				Tim2Flag = 0;
				heart_beat_cnt = (heart_beat_cnt + 1) % 1000;
			}
			
			if (!heart_beat_cnt) {
				Usart_SendByte_communication('#');
				//printf("tik");
			}
				
		}
		
		
        while( control_system::get_system_state() ){//ϵͳ������
            /*---------------------------���ڽ������ݽ���---------------------------*/
			
            if(C__get_PC_parse_flag() == true){                
                printf("PC Message received.\r\n");
                USART_RX_STA = 0;
                C__set_PC_parse_flag(false);
                C__parse_PC_msg();
                
                /*���Ӹ��²�������*/
                sample_period_in_ms = 1000 / control_system::get_Sample_Rate_Hz();
                
                continue;
            }
//            printf("IC_flag: %d\r\n.", C__get_IC_parse_flag());
            

            if(C__get_IC_parse_flag() == true){//ֱ�Ӹ���reference
                printf("IC Message received.\r\n");
                USART_RX_STA = 0;
                C__set_IC_parse_flag(false);
                printf("reference = %#1.6f\r\n", C__parse_IC_msg());
                C__parse_IC_msg();
            }
            if(Tim2Flag==1) {
				Tim2Flag = 0;
				cnt = (cnt + 1) % sample_period_in_ms;
				if (!cnt) {
					
					switch (control_system::get_Is_close_loop()){
						case closed_loop:{
                            

							switch(control_system::get_controller_type()){
								case LADRC:{
                                    
		//                            /*-------------------------------���Բ���---------------------------*/
		//                            auto start = system_clock::now();
		//                            /*-----------------------------�������ײ�---------------------------*/
	//								auto output = ESO::get_output();
	//								printf("output = ");
	//								output.display();
                                    if ( control_system::is_rst_needed() ) {
                                        
                                        control_system::clear_rst_flag();
                                        ctrl.Parameter_init();
                                        ESO_3rd.LADRC_based_current_DESO_init();
                                        
                                    }
									
									
                                    ESO_3rd.Iterate();//�õ���ǰ���

																		
									float u = control_system::update_control_signal_V(ctrl.Iterate(ESO::get_output(), control_system::get_reference())[0][0]);
									
									auto info_res = pack_to_send(ctrl);
									my_usart_send_sys_state(&info_res);
									
	//								printf("reference = %f\r\n", control_system::get_reference());
	//								auto ctrl_iter = ctrl.Iterate(output, control_system::get_reference());
	//								printf("ctrl_iter =\r\n");
	//								ctrl_iter.display();
									//printf("get_sensor_voltage_V = %#1.6f\r\n", control_system::get_sensor_voltage_V());
	//								
	//                                printf("Control Signal = %#1.6f V\r\n", u);

									/*-------------------------------���Բ���---------------------------*/
		//                            auto end   = system_clock::now();
		//                            auto duration = duration_cast<microseconds>(end - start);
		//                            printf("%f us spent.\r\n", double(duration.count()));
									/*-----------------------------�������ײ�---------------------------*/
									break;
								}
								case PID:{
                                    if ( control_system::is_rst_needed() ) {
                                        
                                        control_system::clear_rst_flag();
                                        ctrl.Parameter_init();
                                    }
                                    
                                    float u = control_system::update_control_signal_V(ctrl.Iterate(ESO::get_output(), control_system::get_reference())[0][0]);
                                    
                                    auto info_res = pack_to_send(ctrl);
									my_usart_send_sys_state(&info_res);
                                    //printf("get_sensor_voltage_V = %#1.6f\r\n", control_system::get_sensor_voltage_V());
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
            }
				
        }//while(sys.get_system_state() )
        /*------------------���³�ʼ��������----------------*/
        
    }//while(1)

}






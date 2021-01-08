//
// Created by Frank Young on 2020/3/19.
//

#include "objects.h"
#include "deadzone_compensation.h"



bool control_system::sys_running_state          = true;
float control_system::Sample_Rate_of_Sensor_Hz  = 100;
float control_system::Sample_Rate_Hz            = 100;
float control_system::LADRC_wc                  = 10;
float control_system::LADRC_wo                  = 100;
float control_system::LADRC_b0                  = 60;
float control_system::LADRC_wc_bar              = 4;
bool control_system::Is_close_loop              = true;
uint8_t control_system::controller_type         = 0x80;
uint8_t control_system::open_loop_input_type    = 0x80;
uint8_t control_system::run_time                = 0;
float control_system::PID_Kp                    = 100;
float control_system::PID_Ki                    = 1;
float control_system::PID_Kd                    = 0;
float control_system::open_loop_input_sine_amp  = 0;
float control_system::open_loop_input_sine_freq = 0;
float control_system::open_loop_input_step_amp  = 0;
float control_system::open_loop_input_step_time = 0;
float control_system::deadzone_compensation_dac1= 0;
float control_system::deadzone_compensation_dac2= 0;
float control_system::reference                 = 1;
float control_system::sensor_voltage_V          = 0;
float control_system::control_signal_V          = 0;
bool control_system::reset_flag                 = false;

float ESO::compensation_signal					= 0;
float controller::sigma 						= 1;
float controller::ratio							= 1;
float controller::bl							= 0;
float controller::br							= 0;
Matrix ESO::yd(3, 1);

float control_system::kalman_Q = 0.5;
float control_system::kalman_R = 25;

kalman control_system::klm(kalman_Q, kalman_R, 0);

float ESO::st_need_acc_threashold = 0.1;
float ESO::st_disturbance_est_gain = 1;
bool ESO::fl_param_update = false;

bool suspend_flag = false;
error_evaluate ee;

float error_evaluate::iterate(float new_err) {
    curr_avg = beta * curr_avg + (1.0f - beta) * fabs(new_err);
    //printf("avg = %f\r\n", curr_avg);
    if (fabs(curr_avg) <= threashold) suspend_flag = true;
    else suspend_flag = false;
    return curr_avg;
}

control_system::control_system() {
    feedback_to_ESO = 0;
}

float control_system::update_sensor_voltage_V() {
	sensor_voltage_V = (static_cast<float>(Get_Adc_Average(3, 10)) / 4095.0f) * 3.3f;
	sensor_voltage_V = klm.iterate(sensor_voltage_V);
	//printf("sensor_voltage_V = %f\r\n", sensor_voltage_V);
	return sensor_voltage_V;
}


//vector<vector<float> > control_system::get_Input_for_ESO(const string &ESO_type) {//获取ESO的输入信号ud
//    vector<vector<float> > ESO_Input;
//    if(ESO_type == "Origin_3_orders"){
//        ESO_Input.push_back({control_signal_V, sensor_voltage_V});
//    }
//    return ESO_Input;
//}

ESO::ESO(uint8_t order) : disturbance_est_gain(0.0f), fl_need_acc(false), avg_speed_est(0.0f), new_sample_weight(0.05f), need_acc_threashold(0.1f) {
    observer_order = order;
    A = Matrix(observer_order, observer_order);
    B = Matrix(observer_order, 2);
    C = Matrix(order, order);
    D = Matrix(order, 2);
    acc_A = Matrix(observer_order, observer_order);
    acc_B = Matrix(observer_order, 2);
    acc_C = Matrix(order, order);
    acc_D = Matrix(order, 2);
    Lc=Matrix(order, 1);
    acc_Lc=Matrix(order, 1);
    Z =Matrix(order, 1);
    ud=Matrix(2,1);
    
}

uint8_t ESO::LADRC_based_current_DESO_init() {
	beta = expf(-control_system::LADRC_wo*(1.0f/control_system::Sample_Rate_Hz));
    acc_beta = disturbance_est_gain * beta;
    
    Matrix phi{{1.0f, (1.0f/control_system::Sample_Rate_Hz), powf(control_system::Sample_Rate_Hz, -2)*0.5f},
               {0.0f, 1.0f, (1.0f/control_system::Sample_Rate_Hz)},
               {0.0f, 0.0f, 1.0f}};
    Lc = {{1.0f - powf(beta,3)},
          {1.5f * (powf(beta,3) - powf(beta, 2) - beta + 1.0f) * control_system::Sample_Rate_Hz},
          {powf((1.0f - beta), 3) * powf(control_system::Sample_Rate_Hz, 2)}};
    acc_Lc = {{1.0f - powf(acc_beta,3)},
          {1.5f * (powf(acc_beta,3) - powf(acc_beta, 2) - acc_beta + 1) * control_system::Sample_Rate_Hz},
          {powf((1-acc_beta),3) * powf(control_system::Sample_Rate_Hz,2)}};
    
    Matrix Lp, acc_Lp;
    Lp = phi * Lc;
    acc_Lp = phi * acc_Lc;
    Matrix gamma{{control_system::LADRC_b0*powf(control_system::Sample_Rate_Hz, -2)*0.5f},
                 {control_system::LADRC_b0*(1.0f/control_system::Sample_Rate_Hz)},
                 {0.0f}};     
    Matrix H{{1.0f, 0.0f, 0.0f}};
    Matrix J{{0.0f}};
    Matrix I_3rd{{1.0f,0.0f,0.0f},{0.0f,1.0f,0.0f},{0.0f,0.0f,1.0f}};
    A = phi-(Lp*H);
    B = Matrix::cat(1, gamma-Lp*J, Lp);
    C = I_3rd-Lc*H;
    D = Matrix::cat(1,Matrix(3,1)-Lc*J, Lc);
    acc_A = phi-(acc_Lp*H);
    acc_B = Matrix::cat(1, gamma-acc_Lp*J, acc_Lp);
    acc_C = I_3rd-acc_Lc*H;
    acc_D = Matrix::cat(1,Matrix(3,1)-acc_Lc*J, acc_Lc);
    return 0;
}

float ESO::set_disturbance_est_gain(float deg) {
    disturbance_est_gain = deg;
    LADRC_based_current_DESO_init();
    return deg;
}

Matrix ESO::get_(char Mat) {
    switch(Mat) {
        case 'A':
            return A;
        case 'B':
            return B;
        case 'C':
            return C;
        case 'D':
            return D;
        case 'L':
            return Lc;
        case 'Z':
            return Z;
        case 'u':
            return ud;
        case 'y':
            return yd;
        default:
            printf("ESO::get_ Error: Wrong input.");
            exit(0);
    }
}

uint8_t ESO::set_Init_state(const Matrix& Z0) {
    Z = Z0;
    return 0;
}

Matrix ESO::Iterate() {
    Matrix Z_next;
	auto sensor = control_system::update_sensor_voltage_V();
    switch(control_system::controller_type){
        case LADRC:{
			//control_system::update_sensor_voltage_V();
//            ud = {{control_system::control_signal_V}, {control_system::sensor_voltage_V}};
            //printf("%x\r\n", suspend_flag);
            
            if ( !suspend_flag ) {                
                ud = {{control_system::get_control_signal_V() + compensation_signal}, {sensor}};
                /*更新参数*/
                check_for_param_update();
                /*计算并更新速度估计平均*/
                avg_speed_est = (1 - new_sample_weight) * avg_speed_est + new_sample_weight * yd[1][0];
                
                if ( fabs(avg_speed_est) < need_acc_threashold)
                    fl_need_acc = true;
                else
                    fl_need_acc = false;
                
                if ( !fl_need_acc ) {
                    Z_next = (A*Z) + (B*ud);
                    yd = (C*Z) + (D*ud);
                } else {
                    
                    Z_next = (acc_A*Z) + (acc_B*ud);
                    yd = (acc_C*Z) + (acc_D*ud);
                }
                
                Z = Z_next;
                
            }
//			printf("A  = \r\n");
//			A.display();
//			printf("B = \r\n");
//			B.display();
//			printf("C = \r\n");
//			C.display();
//			printf("D = \r\n");
//			D.display();
//			printf("Z = \r\n");
//			Z.display();
            return yd;
        }
        case LADRC_decouple:{
            return yd;
        }
        default: return yd;
    }
}

void ESO::set_compensation_signal(const float sig) {
	compensation_signal = sig;
}

float ESO::set_st_need_acc_threashold(const float &need_acc_threashold_) {
    fl_param_update = true;
    return st_need_acc_threashold = need_acc_threashold_;
}

float ESO::set_st_disturbance_est_gain(const float &disturbance_est_gain_) { 
    fl_param_update = true;
    return st_disturbance_est_gain = disturbance_est_gain_;
} 

void ESO::check_for_param_update() {
    if ( fl_param_update ) {
        fl_param_update = false;
        need_acc_threashold = st_disturbance_est_gain;
        disturbance_est_gain = st_disturbance_est_gain;
        LADRC_based_current_DESO_init();        
    }
}

controller::controller() : tp(){
    LADRC_Kd = 0;
    LADRC_Kp = 0;
    Output_Error = 0;
    Control_Signal = 0;
    Transient_profile = 0;
    u0 = 0;
    transfer_mat = {{0, 0, 0, 0}};
}

uint8_t controller::Parameter_init() {
    switch(control_system::controller_type){
        case LADRC:{
			tp.init( control_system::get_LADRC_wc_bar(), 1.0f / control_system::get_Sample_Rate_Hz() );
            Transient_profile = control_system::reference;
            Output_Error = 0;
            LADRC_Kp = (float)pow(control_system::LADRC_wc, 2);
            LADRC_Kd = 2.0f*control_system::LADRC_wc-1.0f;
            transfer_mat = {{LADRC_Kp/control_system::LADRC_b0, -LADRC_Kp/control_system::LADRC_b0, -LADRC_Kd/control_system::LADRC_b0, -1/control_system::LADRC_b0}};
            break;
        }
		case PID: {
//            printf("get_PID_Kp = %f\r\n", control_system::get_PID_Kp());
//            printf("get_PID_Ki = %f\r\n", control_system::get_PID_Ki());
//            printf("get_PID_Kd = %f\r\n", control_system::get_PID_Kd());
//            printf("get_reference = %f\r\n", control_system::get_reference());
			pid.pid_init(control_system::get_PID_Kp(), control_system::get_PID_Ki(), control_system::get_PID_Kd(), control_system::get_reference());
			break;
		}
    }

    return 0;
}

Matrix controller::Iterate(const Matrix& ESO_y, const float& ref) {
	
	
    switch(control_system::controller_type){
        case LADRC:{
            auto sensor = control_system::get_sensor_voltage_V(); // 获取传感器测量值（已滤波）
			Transient_profile = tp.iter( control_system::get_reference() );
			//Output_Error = control_system::get_reference() - control_system::get_sensor_voltage_V();
			Output_Error = Transient_profile - sensor;
            ee.iterate(Output_Error);
			
			
			//Control_Signal = (transfer_mat*(Matrix::cat(2,Matrix{{ref}}, ESO_y)))[0][0];
            //使用ESO观测值
			Control_Signal = (transfer_mat*(Matrix::cat(2,Matrix{{Transient_profile}}, ESO_y)))[0][0];
			
			auto comp_Control_Signal = deadzoneInverse(Control_Signal, mr, br, ml, bl, sigma); //死区开环补偿
			ESO::set_compensation_signal(delta_u(comp_Control_Signal, Transient_profile - ESO_y[0][0], ratio));
			
			auto constrained_ctrl_sig = comp_Control_Signal < 0 ? 
				fmax(comp_Control_Signal, -3.3f) : 
				fmin(comp_Control_Signal, 3.3f);
			Control_Signal = Control_Signal < 0 ?
                fmax(Control_Signal, -3.3f) : fmin(Control_Signal, 3.3f);
			//Control_Signal = constrained_ctrl_sig;
			/*=================================================================*/
			set_output(constrained_ctrl_sig); // 设置DAC电压,并适时关闭控制器
			/*=================================================================*/
            return {{Control_Signal}};
			//return {{constrained_ctrl_sig}};
        }
        case PID:{
            auto sensor = control_system::update_sensor_voltage_V();            
            auto ref = control_system::get_reference();
            
            Transient_profile = tp.iter( ref );
			Output_Error = ref - sensor;
            ee.iterate(Output_Error);
            
			Control_Signal =  pid.IncPIDCalc(sensor, ref);
            //printf("Control_Signal = %f\r\n", Control_Signal);
//            printf("sensor = %f\r\n", sensor);
//            printf("ref = %f\r\n", control_system::get_reference());
			/*=================================================================*/
			set_output(Control_Signal); // 设置DAC电压,并适时关闭控制器
			/*=================================================================*/
            return Matrix{ { Control_Signal } };
        }
        default:{
            printf("controller::Iterate Error: Wrong controller type, \"controller_type\" check is requested.");
            return {{0}};
        }
    }//switch(control_system::controller_type)
}

float controller::get_(const string& member_name) {
    if(member_name == "Kp"){
        return LADRC_Kp;
    }
    else if(member_name == "Kd"){
        return LADRC_Kd;
    }
    else if(member_name == "Output_Error"){
        return Output_Error;
    }
    else if(member_name == "Control_Signal"){
        return Control_Signal;
    }
    else if(member_name == "Transient_profile"){
        return Transient_profile;
    }
    else if(member_name == "u0"){
        return u0;
    }
    else{
        printf("controller::get_ Error: Wrong input content.");
        return 0;
    }

}

float controller::set_ratio(const float& ratio_) {
	return ratio = ratio_;
}

float controller::set_sigma(const float& sigma_) {
	return sigma = sigma_;
}

float controller::set_bl(const float& bl_) {
	return bl = bl_;
}

float controller::set_br(const float& br_) {
	return br = br_;
}


//	float get_ref() {
//		return control_system::get_reference();
//	}
//	float get_transient_profile() {
//		return info_ctrl.get_Transient_profile();
//	}
//	float get_Output_Error() {
//		return info_ctrl.get_Output_Error();
//	}
//	float get_Control_Signal() {
//		return info_ctrl.get_Control_Signal();
//	}
//	float get_ESO_first_order_state() {
//		return ESO::get_output()[0][0];
//	}
//	float get_ESO_second_order_state() {
//		return ESO::get_output()[1][0];
//	}
//	float get_ESO_third_order_state() {
//		return ESO::get_output()[2][0];
//	}
//	float get_ESO_fourth_order_state() {
//		return ESO::get_output()[3][0];
//	}

info &pack_to_send(controller &ctrl) {
	static info info_;
	info_.ref = control_system::get_reference();
	info_.transient_profile = ctrl.get_Transient_profile();
	info_.err = ctrl.get_Output_Error();
	auto yd = ESO::get_output();
	info_.ESO_order1 = yd[0][0];
	info_.ESO_order2 = yd[1][0];
	info_.ESO_order3 = yd[2][0];
	info_.ctrl_sig = ctrl.get_Control_Signal();
	info_.sensor_pos = control_system::get_sensor_voltage_V();
	
//	printf("ref = %f\r\n", info_.ref);
//	printf("transient_profile = %f\r\n", info_.transient_profile);
//	printf("err = %f\r\n", info_.err);
//	printf("ESO_order1 = %f\r\n", info_.ESO_order1);
//	printf("ESO_order2 = %f\r\n", info_.ESO_order2);
//	printf("ESO_order3 = %f\r\n", info_.ESO_order3);
//	printf("ctrl_sig = %f\r\n", info_.ctrl_sig);
//	printf("sensor_pos = %f\r\n", info_.sensor_pos);
//	printf("\r\n");
	
	return info_;
}

bool control_system::is_rst_needed() {
    return reset_flag;
}

void control_system::clear_rst_flag() {
    reset_flag = false;
}

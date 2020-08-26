//
// Created by Frank Young on 2020/3/19.
//

#include "objects.h"

bool control_system::sys_running_state          = true;
float control_system::Sample_Rate_of_Sensor_Hz  = 1000;
float control_system::Sample_Rate_Hz            = 100;
float control_system::LADRC_wc                  = 15;
float control_system::LADRC_wo                  = 45;
float control_system::LADRC_b0                  = 60;
float control_system::LADRC_wc_bar              = 4;
bool control_system::Is_close_loop              = true;
uint8_t control_system::controller_type         = 0x80;
uint8_t control_system::open_loop_input_type    = 0x80;
uint8_t control_system::run_time                = 0;
float control_system::PID_Kp                    = 0;
float control_system::PID_Ki                    = 0;
float control_system::PID_Kd                    = 0;
float control_system::open_loop_input_sine_amp  = 0;
float control_system::open_loop_input_sine_freq = 0;
float control_system::open_loop_input_step_amp  = 0;
float control_system::open_loop_input_step_time = 0;
float control_system::deadzone_compensation_dac1= 0;
float control_system::deadzone_compensation_dac2= 0;
float control_system::reference                 = 0;
float control_system::sensor_voltage_V          = 0;
float control_system::control_signal_V          = 0;
Matrix ESO::yd(3, 1);

control_system::control_system(){
    feedback_to_ESO          = 0;
}

float control_system::update_sensor_voltage_V() {
	sensor_voltage_V = (Get_Adc(3) + 1) / 4096 * 3.3f;
	return sensor_voltage_V;
}


//vector<vector<float> > control_system::get_Input_for_ESO(const string &ESO_type) {//获取ESO的输入信号ud
//    vector<vector<float> > ESO_Input;
//    if(ESO_type == "Origin_3_orders"){
//        ESO_Input.push_back({control_signal_V, sensor_voltage_V});
//    }
//    return ESO_Input;
//}

ESO::ESO(uint8_t order) {
    observer_order = order;
    A = Matrix(observer_order, observer_order);
    B = Matrix(observer_order, 2);
    C = Matrix(order, order);
    D = Matrix(order, 2);
    Lc=Matrix(order, 1);
    Z =Matrix(order, 1);
    ud=Matrix(2,1);
    beta = exp(-control_system::LADRC_wo*(1.0f/control_system::Sample_Rate_Hz));
}

uint8_t ESO::LADRC_based_current_DESO_init() {
    
    Matrix phi{{1.0f, (1.0f/control_system::Sample_Rate_Hz), powf(control_system::Sample_Rate_Hz, -2)*0.5f},
               {0.0f, 1.0f, (1.0f/control_system::Sample_Rate_Hz)},
               {0.0f, 0.0f, 1.0f}};
    Lc = {{1-powf(beta,3)},
          {(2.0f-3.0f*beta+powf(beta,3))*control_system::Sample_Rate_Hz},
          {powf((1-beta),3)*powf(control_system::Sample_Rate_Hz,2)}};
    Matrix Lp;
    Lp = phi*Lc;
    Matrix gamma{{control_system::LADRC_b0*powf(1.0f/control_system::Sample_Rate_Hz, 2)*0.5f},
                 {control_system::LADRC_b0*(1.0f/control_system::Sample_Rate_Hz)},
                 {0.0f}};     
    Matrix H{{1.0f, 0.0f, 0.0f}};
    Matrix J{{0.0f}};
    Matrix I_3rd{{1.0f,0.0f,0.0f},{0.0f,1.0f,0.0f},{0.0f,0.0f,1.0f}};
    A = phi-(Lp*H);
    B = Matrix::cat(1, gamma-Lp*J, Lp);
    C = I_3rd-Lc*H;
    D = Matrix::cat(1,Matrix(3,1)-Lc*J, Lc);
    return 0;
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
	control_system::update_sensor_voltage_V();
    switch(control_system::controller_type){
        case LADRC:{
            ud = {{control_system::control_signal_V}, {control_system::sensor_voltage_V}};
            Z_next = A*Z + B*ud;
            yd = C*Z+D*ud;
            Z = Z_next;
//			printf("A = \r\n");
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

controller::controller() {
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
            Transient_profile = control_system::reference;
            Output_Error = 0;
            LADRC_Kp = (float)pow(control_system::LADRC_wc, 2);
            LADRC_Kd = 2.0f*control_system::LADRC_wc-1.0f;
            transfer_mat = {{LADRC_Kp/control_system::LADRC_b0, -LADRC_Kp/control_system::LADRC_b0, -LADRC_Kd/control_system::LADRC_b0, -1/control_system::LADRC_b0}};
            break;
        }
    }

    return 0;
}

Matrix controller::Iterate(const Matrix& ESO_y, const float& ref) {
    switch(control_system::controller_type){
        case LADRC:{
            return transfer_mat*(Matrix::cat(2,Matrix{{ref}}, ESO_y));
        }
        case PID:{
            return transfer_mat*ESO_y;
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
	return info_;
}


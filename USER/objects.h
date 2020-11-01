//
// Created by Frank Young on 2020/3/19.
//

#ifndef __OBJECTS_H
#define __OBJECTS_H

#include <stdint.h>
//#include <vector>
//#include <iostream>
#include <cmath>
#include <stdio.h>
#include <string>
#include <cmath>
#include "usart.h"
#include "Matrix.h"
#include "adc.h"
#include "dac.h"
#include "info_to_send.h"
#include "transient_profile.h"
#include "kalman.h"


enum controller_name {
    LADRC=0x80,
    PID=0x40,
    LADRC_decouple=0x08
};

enum signal_type {
    sine=0x80,
    step=0x40
};

enum feedback {
    open_loop=0,
    closed_loop=1
};

using namespace std;
class control_system;
class ESO;
class controller;
class communication_protocol;

class control_system {
public:
    
    friend class ESO;
    friend class controller;
    friend class communication_protocol;

    control_system();
    static bool     get_system_state();
    static bool     get_Is_close_loop();
    static uint8_t  set_Is_close_loop(const uint8_t& yon);
    static uint8_t  set_system_state(const uint8_t& system_state);
    static uint8_t  start_running();
    static uint8_t  stop_running();
    static float    get_Sample_Rate_Hz();
    static float    set_Sample_Rate_Hz(const float& fs);
    static float    get_Sample_Rate_of_Sensor_Hz();
    static float    set_Sample_Rate_of_Sensor_Hz(const float& fs);
    static float    get_reference();
    static float    update_reference(const float &ref);
    static float    get_sensor_voltage_V();
    static float    update_sensor_voltage_V();
    static float    get_control_signal_V();
    static float    update_control_signal_V(const float& u);
    static uint8_t  set_run_time(const uint8_t& time);
    static uint8_t  get_controller_type();
    static uint8_t  get_open_loop_input_type();
    static float    set_LADRC_wc(const float& wc);
    static float    set_LADRC_wo(const float& wo);
    static float    set_LADRC_b0(const float& b0);
    static float    set_LADRC_wc_bar(const float& wc_bar);
	static float	get_LADRC_wc_bar() {
		return LADRC_wc_bar;
	}
    static float    set_PID_Kp(const float& kp);
    static float    set_PID_Ki(const float& ki);
    static float    set_PID_Kd(const float& kd);
    static float    set_open_loop_input_sine_amp(const float& amp);
    static float    set_open_loop_input_sine_freq(const float& freq);
    static float    set_open_loop_input_step_amp(const float& amp);
    static float    set_open_loop_input_step_time(const float& time);
    static float    set_deadzone_compensation_dac1(const float& comp);
    static float    set_deadzone_compensation_dac2(const float& comp);
    static uint8_t  set_controller_type(const uint8_t& type);
    static uint8_t  set_open_loop_input_type(const uint8_t & type);
    //vector<vector<float>>   get_Input_for_ESO(const string& ESO_type);//更新ESO的输入

private:
    static bool     sys_running_state;
    static float    Sample_Rate_Hz;
    static float    Sample_Rate_of_Sensor_Hz;
    static float    reference;
    static float    sensor_voltage_V;
    static float    control_signal_V;
    static uint8_t  controller_type;
    static uint8_t  open_loop_input_type;
    static float    LADRC_wc;
    static float    LADRC_wo;
    static float    LADRC_b0;
    static float    LADRC_wc_bar;
    float           feedback_to_ESO;
    static bool     Is_close_loop;
    static uint8_t  run_time;
    static float    PID_Kp;
    static float    PID_Ki;
    static float    PID_Kd;
    static float    open_loop_input_sine_amp;
    static float    open_loop_input_sine_freq;
    static float    open_loop_input_step_amp;
    static float    open_loop_input_step_time;
    static float    deadzone_compensation_dac1;
    static float    deadzone_compensation_dac2;
	
	static struct kalman klm;
	static float kalman_Q;
	static float kalman_R;
};

class ESO{
public:
    explicit ESO(uint8_t order);

//    template <typename T>
//    vector<vector<T> > Mat_product(vector<vector<T>> &ans, const vector<vector<T>> &X, const vector<vector<T>> &Y);
//
//    template <typename T>
//    vector<vector<T> > Mat_plus(vector<vector<T>> &ans, const vector<vector<T>> &X, const vector<vector<T>> &Y);
//
//    template <typename T>
//    vector<vector<T> > Mat_minus(vector<vector<T>> &ans, const vector<vector<T>> &X, const vector<vector<T>> &Y);
//
//    template <typename T>
//    vector<vector<T> > Mat_cat(uint8_t type, vector<vector<T>> &ans, const vector<vector<T>> &X, const vector<vector<T>> &Y);
//
//    template <typename T>
//    uint8_t Mat_print(const string& str, const vector<vector<T>> &X);
//
//    template <typename T>
//    vector<vector<T> > Mat_copy(vector<vector<T>> &blank, const vector<vector<T>> &obj);

    uint8_t                         LADRC_based_current_DESO_init();
    Matrix                          get_(char Mat);
    uint8_t                         set_Init_state(const Matrix& Z0);
    Matrix                          Iterate();
    static Matrix                   get_output();
	static void 					set_compensation_signal(const float sig);

private:
    uint8_t    observer_order;
    Matrix A;
    Matrix B;
    Matrix C;
    Matrix D;
    Matrix Lc;
    Matrix Z;
    Matrix ud;
    static Matrix yd; // ESO估计值
	static float compensation_signal; // 死区偏差补偿
    float beta;
};

class controller {
public:
    friend class communication_protocol;

    controller();
    uint8_t     Parameter_init();
    Matrix      Iterate(const Matrix& ESO_, const float& ref);
    float       get_(const string& member_name);
	float get_Transient_profile() {
		return Transient_profile;
	}
	float get_Output_Error() {
		return Output_Error;
	}
	float get_Control_Signal() {
		return Control_Signal;
	}
	static float set_ratio(const float& ratio_);
	static float set_sigma(const float& sigma_);
	static float set_bl(const float& bl_);
	static float set_br(const float& br_);
	
private:
    float LADRC_Kp;
    float LADRC_Kd;
    float Output_Error;
    float Control_Signal;
    float Transient_profile;
    float u0;
    Matrix transfer_mat;
	transient_profile tp;

	static float bl;
	static float br;
	float ml = 1.0;
	float mr = 1.0;
	static float sigma;
	static float ratio; // ESO补偿信号的比例系数

    //Matrix ESO_Output;
};

inline uint8_t control_system::set_system_state(const uint8_t& system_state) {
    return sys_running_state = system_state;
}
inline uint8_t control_system::set_Is_close_loop(const uint8_t& yon) {
    return Is_close_loop = yon;
}
inline uint8_t control_system::set_controller_type(const uint8_t &type) {
    return controller_type = type;
}
inline uint8_t control_system::set_open_loop_input_type(const uint8_t &type) {
    return open_loop_input_type = type;
}
inline float control_system::set_Sample_Rate_Hz(const float& fs){
    return Sample_Rate_Hz = fs;
}
inline float control_system::set_Sample_Rate_of_Sensor_Hz(const float& fs){
    return Sample_Rate_of_Sensor_Hz = fs;
}
inline uint8_t control_system::set_run_time(const uint8_t& time) {
    return run_time = time;
}
inline float control_system::set_LADRC_wc(const float& wc) {
    return LADRC_wc = wc;
}
inline float control_system::set_LADRC_wo(const float& wo) {
    return LADRC_wo = wo;
}
inline float control_system::set_LADRC_b0(const float& b0) {
    return LADRC_b0 = b0;
}
inline float control_system::set_LADRC_wc_bar(const float &wc_bar) {
    return LADRC_wc_bar = wc_bar;
}
inline float control_system::set_PID_Kp(const float &kp) {
    return PID_Kp = kp;
}
inline float control_system::set_PID_Ki(const float &ki) {
    return PID_Ki = ki;
}
inline float control_system::set_PID_Kd(const float &kd) {
    return PID_Kd = kd;
}
inline float control_system::set_open_loop_input_sine_amp(const float &amp) {
    return open_loop_input_sine_amp = amp;
}
inline float control_system::set_open_loop_input_sine_freq(const float &freq) {
    return open_loop_input_sine_freq = freq;
}
inline float control_system::set_open_loop_input_step_amp(const float &amp) {
    return open_loop_input_step_amp = amp;
}
inline float control_system::set_open_loop_input_step_time(const float &time) {
    return open_loop_input_step_time = time;
}
inline float control_system::set_deadzone_compensation_dac1(const float &comp) {
    deadzone_compensation_dac1 = comp;
	return controller::set_bl(comp);
}
inline float control_system::set_deadzone_compensation_dac2(const float &comp) {
    deadzone_compensation_dac2 = comp;
	return controller::set_br(comp);
}
inline bool control_system::get_system_state() {
    return sys_running_state;
}
inline bool control_system::get_Is_close_loop() {
    return Is_close_loop;
}
inline uint8_t control_system::get_controller_type() {
    return controller_type;
}
inline uint8_t control_system::get_open_loop_input_type() {
    return open_loop_input_type;
}
inline float control_system::update_reference(const float &ref) {
    return reference = ref;
}
inline uint8_t control_system::start_running(){
    return sys_running_state = true;
}
inline uint8_t control_system::stop_running(){
    return sys_running_state = false;
}
inline float control_system::get_Sample_Rate_Hz(){
    return Sample_Rate_Hz;
}
inline float control_system::get_Sample_Rate_of_Sensor_Hz(){
    return Sample_Rate_of_Sensor_Hz;
}
inline float control_system::get_reference(){
    return reference;
}
inline float control_system::get_sensor_voltage_V(){
    return sensor_voltage_V;
}
inline float control_system::get_control_signal_V(){
    return control_signal_V;
}

//template<typename T>
//vector<vector<T> > ESO::Mat_product(vector<vector<T> > &ans, const vector<vector<T> > &X, const vector<vector<T> > &Y) {
//    uint8_t column_A = X.size();
//    uint8_t row_A    = X[0].size();
//    uint8_t column_B = Y.size();
//    uint8_t row_B    = Y[0].size();
//    if(column_A!=row_B){
//        if(column_A*row_A==1){
//            ans.resize(column_B, vector<T>(row_B,0));
//            for(uint8_t cB = 0; cB < column_B; ++cB){
//                for(uint8_t rB = 0; rB < row_B; ++rB){
//                    ans[cB][rB] = X[0][0]*Y[cB][rB];
//                }
//            }
//        }
//        else if(column_B*row_B==1){
//            ans.resize(column_A, vector<T>(row_A,0));
//            for(uint8_t cA = 0; cA < column_B; ++cA){
//                for(uint8_t rA = 0; rA < row_B; ++rA){
//                    ans[cA][rA] = X[cA][rA]*Y[0][0];
//                }
//            }
//        }
//        else{
//            ans.resize(0, vector<T>(0));
//            printf("ESO::Mat_product Error: Mismatch of input matrices degree.\r\n");
//        }
//        return ans;
//    }
//    ans.resize(column_B, vector<T>(row_A,0));
//    for(uint8_t cB = 0; cB < column_B; ++cB){
//        for(uint8_t rA = 0; rA < row_A; ++rA){
//            T tmp = 0;
//            for(uint8_t cA = 0; cA < column_A; ++cA){
//                tmp += X[cA][rA]*Y[cB][cA];
//            }
//            ans[cB][rA] = tmp;
//        }
//    }
//    return ans;
//}
//
//template<typename T>
//vector<vector<T> > ESO::Mat_plus(vector<vector<T> > &ans, const vector<vector<T> > &X, const vector<vector<T> > &Y){
//    uint8_t column_A = X.size();
//    uint8_t row_A    = X[0].size();
//    uint8_t column_B = Y.size();
//    uint8_t row_B    = Y[0].size();
//    if(!(column_A == column_B && row_A == row_B)){
//        ans.resize(0, vector<T>(0));
//        printf("ESO::Mat_plus Error: Mismatch of input matrices degree.\r\n");
//        return ans;
//    }
//    ans.resize(column_A, vector<T>(row_A, 0));
//    for(uint8_t m=0; m<column_A; ++m){
//        for(uint8_t n=0; n<row_A; ++n){
//            ans[m][n] = X[m][n] + Y[m][n];
//        }
//    }
//    return ans;
//}
//
//template<typename T>
//vector<vector<T> > ESO::Mat_minus(vector<vector<T> > &ans, const vector<vector<T> > &X, const vector<vector<T> > &Y){
//    uint8_t column_A = X.size();
//    uint8_t row_A    = X[0].size();
//    uint8_t column_B = Y.size();
//    uint8_t row_B    = Y[0].size();
//    if(!(column_A == column_B && row_A == row_B)){
//        ans.resize(0, vector<T>(0));
//        printf("ESO::Mat_minus Error: Mismatch of input matrices degree.\r\n");
//        return ans;
//    }
//    ans.resize(column_A, vector<T>(row_A, 0));
//    for(uint8_t m=0; m<column_A; ++m){
//        for(uint8_t n=0; n<row_A; ++n){
//            ans[m][n] = X[m][n] - Y[m][n];
//        }
//    }
//    return ans;
//}
//
//template<typename T>
//vector<vector<T>>
//ESO::Mat_cat(uint8_t type, vector<vector<T>> &ans, const vector<vector<T>> &X, const vector<vector<T>> &Y) {
//    uint8_t column_A = X.size();
//    uint8_t row_A    = X[0].size();
//    uint8_t column_B = Y.size();
//    uint8_t row_B    = Y[0].size();
//    if(type == 1){
//        if(row_A != row_B){
//            printf("ESO::Mat_cat Error: Input Matrices with different row numbers. Exit processing.\r\n");
//            ans.resize(0, vector<T>(0));
//            return ans;
//        }
//        ans.resize(0, vector<T>(0));
//        for(uint8_t n=0; n<column_A; ++n){
//            ans.push_back(X[n]);
//        }
//        for(uint8_t n=0; n<column_B; ++n){
//            ans.push_back(Y[n]);
//        }
//    }
//    else if(type == 2){
//        if(column_A != column_B){
//            printf("ESO::Mat_cat Error: Input Matrices with different column numbers. Exit processing.\r\n");
//            ans.resize(0, vector<T>(0));
//            return ans;
//        }
//        ans.resize(0, vector<T>(0));
//        for(uint8_t i=0; i<column_A; ++i){
//            vector<T> tmp(0);
//            for(uint8_t n=0; n<(row_A+row_B); ++n){
//                tmp.resize(0);
//                if(n<row_A){
//                    for(auto m:X[n]){
//                        tmp.push_back(m);
//                    }
//                }
//                else{
//                    for(auto m:Y[n-row_A]){
//                        tmp.push_back(m);
//                    }
//                }
//            }
//            ans.push_back(tmp);
//        }
//    }
//    else{
//        printf("ESO::Mat_cat Error: Wrong input type. Exit processing.\r\n");
//        ans.resize(0, vector<T>(0));
//        return ans;
//    }
//    return ans;
//}
//
//template<typename T>
//uint8_t ESO::Mat_print(const string& str, const vector<vector<T>> &X) {
//    uint8_t column_A = X.size();
//    uint8_t row_A    = X[0].size();
//    printf("%s =\r\n", str.c_str());
//    for(uint8_t i=0; i<row_A; ++i){
//        for(uint8_t j=0; j<column_A; ++j){
//            printf("%f", X[j][i]);
//            printf(", ");
//        }
//        printf("\r\n");
//    }
//    return 0;
//}
//
//template<typename T>
//vector<vector<T>> ESO::Mat_copy(vector<vector<T>> &blank, const vector<vector<T>> &obj) {
//    uint8_t column = obj.size();
//    uint8_t row    = obj[0].size();
//    blank.resize(column, vector<T>(row));
//    for(uint8_t i=0; i<column; ++i){
//        for(uint8_t j=0; j<row; ++j){
//            blank[i][j] = obj[i][j];
//        }
//    }
//    return blank;
//}

inline Matrix ESO::get_output() {
    return yd;
}
inline float control_system::update_control_signal_V(const float& u){
	set_output(u);
    return control_signal_V = u;
}

info &pack_to_send(controller &ctrl);

#endif //WEEDER_CONTROLLER_OBJECTS_H

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
#include "usart.h"
#include "Matrix.h"
#include "adc.h"
#include "dac.h"
#include "info_to_send.h"
#include "transient_profile.h"
#include "kalman.h"
#include "pid_controller.h"


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
	static float	get_PID_Kp();
    static float    set_PID_Ki(const float& ki);
	static float	get_PID_Ki();
    static float    set_PID_Kd(const float& kd);
	static float	get_PID_Kd();
    static float    set_open_loop_input_sine_amp(const float& amp);
    static float    set_open_loop_input_sine_freq(const float& freq);
    static float    set_open_loop_input_step_amp(const float& amp);
    static float    set_open_loop_input_step_time(const float& time);
    static float    set_deadzone_compensation_dac1(const float& comp);
    static float    set_deadzone_compensation_dac2(const float& comp);
    static uint8_t  set_controller_type(const uint8_t& type);
    static uint8_t  set_open_loop_input_type(const uint8_t & type);
    
    static bool is_rst_needed();
    static void clear_rst_flag();
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
    
    static bool reset_flag;
    
};

class ESO{
public:
    explicit ESO(uint8_t order);

    uint8_t                         LADRC_based_current_DESO_init();
    Matrix                          get_(char Mat);
    uint8_t                         set_Init_state(const Matrix& Z0);
    Matrix                          Iterate();
    static Matrix                   get_output();
	static void 					set_compensation_signal(const float sig);
    float                           set_disturbance_est_gain(float deg);
    
    static float                    set_st_need_acc_threashold(const float&);
    static float                    set_st_disturbance_est_gain(const float&);
    //static void                     set_fl_param_update(bool fl);
    void check_for_param_update();
          

private:
    uint8_t    observer_order;
    Matrix A;
    Matrix B;
    Matrix C;
    Matrix D;
    /*跨越死区时采用的矩阵*/
    Matrix acc_A;
    Matrix acc_B;
    Matrix acc_C;
    Matrix acc_D;
    
    Matrix Lc;
    Matrix acc_Lc;
    Matrix Z;
    Matrix ud;
    static Matrix yd; // ESO估计值
	static float compensation_signal; // 死区偏差补偿
    float beta;
    float acc_beta;

    /*根据ESO的速度估计判断被控对象是否处在死区*/
    bool fl_need_acc;
    float avg_speed_est;
    float new_sample_weight;
    float need_acc_threashold;
    static float st_need_acc_threashold;
    
    /*lc_3增益*/
    float disturbance_est_gain;
    static float st_disturbance_est_gain;
    
    /*方便更新参数加入的中间层标志*/
    static bool fl_param_update;
    
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
    
    void check_for_update();

public:    
    transient_profile tp;
	
private:
    float LADRC_Kp;
    float LADRC_Kd;
    float Output_Error;
    float Control_Signal;
    float Transient_profile;
    float u0;
    Matrix transfer_mat;
	
	static float bl;
	static float br;
	float ml = 1.0;
	float mr = 1.0;
	static float sigma;
	static float ratio; // ESO补偿信号的比例系数

	
	class PID pid;

    //Matrix ESO_Output;
};

inline uint8_t control_system::set_system_state(const uint8_t& system_state) {
    return sys_running_state = system_state;
}
inline uint8_t control_system::set_Is_close_loop(const uint8_t& yon) {
    return Is_close_loop = yon;
}
inline uint8_t control_system::set_controller_type(const uint8_t &type) {
    if (controller_type != type)
        reset_flag = true;
    return controller_type = type;
}
inline uint8_t control_system::set_open_loop_input_type(const uint8_t &type) {
    return open_loop_input_type = type;
}
inline float control_system::set_Sample_Rate_Hz(const float& fs){
    if (fs != Sample_Rate_Hz)
        reset_flag = true;
    return Sample_Rate_Hz = fs;
}
inline float control_system::set_Sample_Rate_of_Sensor_Hz(const float& fs){
    if (fs != Sample_Rate_of_Sensor_Hz)
        reset_flag = true;
    return Sample_Rate_of_Sensor_Hz = fs;
}
inline uint8_t control_system::set_run_time(const uint8_t& time) {
    return run_time = time;
}
inline float control_system::set_LADRC_wc(const float& wc) {
    if (wc != LADRC_wc)
        reset_flag = true;
    return LADRC_wc = wc;
}
inline float control_system::set_LADRC_wo(const float& wo) {
    if (wo != LADRC_wo)
        reset_flag = true;
    return LADRC_wo = wo;
}
inline float control_system::set_LADRC_b0(const float& b0) {
    if (b0 != LADRC_b0)
        reset_flag = true;
    return LADRC_b0 = b0;
}
inline float control_system::set_LADRC_wc_bar(const float &wc_bar) {
    if (wc_bar != LADRC_wc_bar)
        reset_flag = true;
    return LADRC_wc_bar = wc_bar;
}
inline float control_system::set_PID_Kp(const float &kp) {
    if (PID_Kp != kp)
        reset_flag = true;
    return PID_Kp = kp;
}
inline float control_system::get_PID_Kp() {
	return PID_Kp;
}
inline float control_system::set_PID_Ki(const float &ki) {
    if (PID_Ki != ki)
        reset_flag = true;
    return PID_Ki = ki;
}
inline float control_system::get_PID_Ki() {
	return PID_Ki;
}
inline float control_system::set_PID_Kd(const float &kd) {
    if (PID_Kd != kd)
        reset_flag = true;
    return PID_Kd = kd;
}
inline float control_system::get_PID_Kd() {
	return PID_Kd;
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


inline Matrix ESO::get_output() {
    return yd;
}
inline float control_system::update_control_signal_V(const float& u){
	set_output(u);
    return control_signal_V = u;
}

info &pack_to_send(controller &ctrl);

class error_evaluate {
private:
    float beta = 0.975;
    float curr_avg = 0;
    float threashold = 0.02;

public:
    float iterate(float new_err);
};

#endif //WEEDER_CONTROLLER_OBJECTS_H

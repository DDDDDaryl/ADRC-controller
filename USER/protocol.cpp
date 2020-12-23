//
// Created by Frank Young on 2020/3/29.
//
#include "protocol.h"
#include "objects.h"
#include <algorithm>
#include <vector>
using namespace std;

class control_system;

class communication_protocol{
private:
    struct  ID_hash_table;
    struct  IC_MESSAGE;
    struct  PC_MESSAGE;
    union   transit;
    static vector<uint8_t>::iterator it_IC_r;
    static vector<uint8_t>::iterator it_PC_r;
    static vector<uint8_t>::iterator it_PC_s;
    static vector<uint8_t> IC_rec_buf;
    static vector<uint8_t> PC_rec_buf;
    static vector<uint8_t> PC_send_buf;
    static uint16_t rec_buf_size;
    static bool IC_parse_flag;
    static bool PC_parse_flag;
    

public:
//    communication_protocol();
    static bool            ID_hash_table_init();
    static uint8_t         IC_rec_buf_write(const uint8_t& Res, uint16_t USART_RX_STA);
    static uint8_t         PC_rec_buf_write(const uint8_t& Res, uint16_t USART_RX_STA);
    static uint8_t         PC_send_buf_write(const uint8_t& data);
    static bool            set_IC_parse_flag(const bool& stat);
    static bool            set_PC_parse_flag(const bool& stat);
    static bool            get_IC_parse_flag();
    static bool            get_PC_parse_flag();
    static float           parse_IC_msg();
    static bool            parse_PC_msg();
    static void            init();

};
struct ID_func{
    explicit ID_func(uint8_t dft = 0, uint8_t (*flag)(const uint8_t&) = nullptr, float (*data)(const float&) = nullptr){
        //ID = dft;
        func_ptr_flag = flag;
        func_ptr_data = data;
    }
    //uint8_t ID;
    float (*func_ptr_data)(const float&);
    uint8_t (*func_ptr_flag)(const uint8_t&);
};



uint16_t communication_protocol::rec_buf_size = 200;
vector<uint8_t> communication_protocol::IC_rec_buf(communication_protocol::rec_buf_size);
vector<uint8_t> communication_protocol::PC_rec_buf(communication_protocol::rec_buf_size);
vector<uint8_t> communication_protocol::PC_send_buf(200);
vector<uint8_t>::iterator communication_protocol::it_IC_r = IC_rec_buf.begin();
vector<uint8_t>::iterator communication_protocol::it_PC_r = PC_rec_buf.begin();
vector<uint8_t>::iterator communication_protocol::it_PC_s = PC_send_buf.begin();

struct communication_protocol::ID_hash_table{
    static ID_func leaf;
    static vector<ID_func> ID_tree;
    //static ID_func& ID_trace(uint8_t ID);
};

inline bool communication_protocol::get_IC_parse_flag() {
    return IC_parse_flag;
}
inline bool communication_protocol::get_PC_parse_flag() {
    return PC_parse_flag;
}
inline bool communication_protocol::set_IC_parse_flag(const bool& stat) {
    return IC_parse_flag = stat;
}
inline bool communication_protocol::set_PC_parse_flag(const bool &stat) {
    return PC_parse_flag = stat;
}
inline uint8_t communication_protocol::IC_rec_buf_write(const uint8_t& Res, uint16_t USART_RX_STA) {
    return IC_rec_buf[USART_RX_STA&0X3FFFu] = Res;
}
inline uint8_t communication_protocol::PC_rec_buf_write(const uint8_t& Res, uint16_t USART_RX_STA) {
    return PC_rec_buf[USART_RX_STA&0X3FFFu] = Res;
}

void communication_protocol::init() {
    rec_buf_size = 200;
    IC_rec_buf.resize(rec_buf_size);
    PC_rec_buf.resize(rec_buf_size);
    PC_send_buf.resize(rec_buf_size);
    it_IC_r = IC_rec_buf.begin();
    it_PC_r = PC_rec_buf.begin();
    it_PC_s = PC_send_buf.begin();
}

ID_func communication_protocol::ID_hash_table::leaf{0, nullptr, nullptr};
vector<ID_func> communication_protocol::ID_hash_table::ID_tree(256, leaf);
bool communication_protocol::IC_parse_flag = false;
bool communication_protocol::PC_parse_flag = false;

struct communication_protocol::IC_MESSAGE{
    float   reference;
    uint8_t body_length;
    explicit IC_MESSAGE(uint8_t length = 0, float ref = 0){
        body_length = length;
        reference = ref;
    }
};

struct communication_protocol::PC_MESSAGE{
    uint8_t body_length;
    explicit PC_MESSAGE(uint8_t length = 0){
        body_length = length;
    }
};

union communication_protocol::transit{
    float d{0};
    uint8_t data[4];
    explicit transit(float ini = 0){
        d = ini;
    }
};

//communication_protocol::communication_protocol() {
//    rec_buf_size = 200;
//    IC_rec_buf.resize(rec_buf_size);
//    PC_rec_buf.resize(rec_buf_size);
//    PC_send_buf.resize(rec_buf_size);
//    it_IC_r = IC_rec_buf.begin();
//    it_PC_r = PC_rec_buf.begin();
//    it_PC_s = PC_send_buf.begin();
//}

uint8_t communication_protocol::PC_send_buf_write(const uint8_t& data) {
    return 0;
}

float communication_protocol::parse_IC_msg() {
    it_IC_r = IC_rec_buf.begin();
    
    while (it_IC_r != IC_rec_buf.end() && *it_IC_r != 0xeb) {
		++it_IC_r;
	}
    
    if(*it_IC_r != 0xeb){
        printf("Invalid Header.\r\n");
        return false;
    }
    else{
        if(*++it_IC_r!=0x90){
            printf("Invalid Header.\r\n");
            return false;
        }
    }
    IC_MESSAGE msg;
    transit trs;
    IC_MESSAGE *p_msg = &msg;
    transit *p_tmp = &trs;
    p_msg->body_length = *++it_IC_r;
    for(uint8_t& p : p_tmp->data){
        p = *++it_IC_r;
    }
    return control_system::update_reference(p_tmp->d);
}
/*默认解析来自PC的参数调试数据时停止控制
 * 若收到数据时系统为停止运行状态，只需调整参数；
 * 若收到数据时系统为正在运行状态，须在设置参数后停止运行，并重新执行控制器初始化
 * 系统停止运行后，只需将system_state用协议置1即可恢复运行状态*/
bool communication_protocol::parse_PC_msg() {
    
    it_PC_r = PC_rec_buf.begin();
//	for (auto it = PC_rec_buf.begin(); it != PC_rec_buf.end(); ++it)
//		printf("%02x ", *it);
//	printf("\r\n");
	
	while (it_PC_r != PC_rec_buf.end() && *it_PC_r != 0xeb) {
		++it_PC_r;
	}
    if(*it_PC_r != 0xeb){
        printf("Invalid Header.\r\n");
		fill(PC_rec_buf.begin(), PC_rec_buf.end(), 0);
        return false;
    }
    else{
        if(*++it_PC_r!=0x90){
            printf("Invalid Header.\r\n");
			fill(PC_rec_buf.begin(), PC_rec_buf.end(), 0);
            return false;
        }
    }
//    auto p_msg = new PC_MESSAGE;
//    auto p_tmp = new transit;
    PC_MESSAGE msg;
    transit trans;
    auto p_msg = &msg;
    auto p_tmp = &trans;

    control_system::set_system_state(false);
    p_msg->body_length = *++it_PC_r; //data length(self included)
    uint8_t ID_tmp = 0;

    for(uint8_t i=1; i<p_msg->body_length; ++i){
        ID_tmp = *++it_PC_r;
        if(ID_tmp & 0x80u){ // ID 最高位代表数据类型
            for(int j = 0;  j < 4; ++j){ // 
                p_tmp->data[j] = *++it_PC_r;
                ++i;
				//printf("j = %x\r\n", j);
            }
			auto res = ID_hash_table::ID_tree[ID_tmp].func_ptr_data(p_tmp->d);
            printf("ID_tmp = %#04x, data = %1.2f\r\n", ID_tmp, res);
        }
        else{
            auto ret = ID_hash_table::ID_tree[ID_tmp].func_ptr_flag(*++it_PC_r);
            ++i;
            printf("ID_tmp = %#04x, data = %#04x\r\n", ID_tmp, ret );
        }
    }
//    delete p_msg;
//    delete p_tmp;
	fill(PC_rec_buf.begin(), PC_rec_buf.end(), 0);
    return true;
}

enum ID_table {
	sys_running_state	        = 0x09,
	Is_close_loop	            = 0x0A,
	controller_type	            = 0x0B,
	open_loop_input_type	    = 0x0C,
	Sample_Rate_Hz	            = 0x8C,
	Sample_Rate_of_Sensor_Hz	= 0x8D,
	run_time	                = 0x0E,
	reference_signal			= 0xCF,
	LADRC_wc	                = 0x91,
	LADRC_wo	                = 0x92,
	LADRC_b0	                = 0x93,
	LADRC_wc_bar	            = 0x94,
	PID_Kp	                    = 0x99,
	PID_Ki	                    = 0x9A,
	PID_Kd	                    = 0x9B,
	open_loop_input_sine_amp	= 0xB1,
	open_loop_input_sine_freq	= 0xB2,
	open_loop_input_step_amp	= 0xB9,
	open_loop_input_step_time	= 0xBA,
	deadzone_compensation_dac1	= 0xC9,
	deadzone_compensation_dac2	= 0xD1,
	ratio						= 0x95,
	sigma						= 0x96,
    st_disturbance_est_gain     = 0x97,
    st_need_acc_threashold      = 0xA1
};

bool communication_protocol::ID_hash_table_init() {

    communication_protocol::ID_hash_table::ID_tree[0x09].func_ptr_flag = &control_system::set_system_state;
    communication_protocol::ID_hash_table::ID_tree[0x0A].func_ptr_flag = &control_system::set_Is_close_loop;
    communication_protocol::ID_hash_table::ID_tree[0x0B].func_ptr_flag = &control_system::set_controller_type;
    communication_protocol::ID_hash_table::ID_tree[0x0C].func_ptr_flag = &control_system::set_open_loop_input_type;
    communication_protocol::ID_hash_table::ID_tree[0x8C].func_ptr_data = &control_system::set_Sample_Rate_Hz;
    communication_protocol::ID_hash_table::ID_tree[0x8D].func_ptr_data = &control_system::set_Sample_Rate_of_Sensor_Hz;
    communication_protocol::ID_hash_table::ID_tree[0x0E].func_ptr_flag = &control_system::set_run_time;
	communication_protocol::ID_hash_table::ID_tree[reference_signal].func_ptr_data = &control_system::update_reference;
    communication_protocol::ID_hash_table::ID_tree[0x91].func_ptr_data = &control_system::set_LADRC_wc;
    communication_protocol::ID_hash_table::ID_tree[0x92].func_ptr_data = &control_system::set_LADRC_wo;
    communication_protocol::ID_hash_table::ID_tree[0x93].func_ptr_data = &control_system::set_LADRC_b0;
    communication_protocol::ID_hash_table::ID_tree[0x94].func_ptr_data = &control_system::set_LADRC_wc_bar;
    communication_protocol::ID_hash_table::ID_tree[0x99].func_ptr_data = &control_system::set_PID_Kp;
    communication_protocol::ID_hash_table::ID_tree[0x9A].func_ptr_data = &control_system::set_PID_Ki;
    communication_protocol::ID_hash_table::ID_tree[0x9B].func_ptr_data = &control_system::set_PID_Kd;
    communication_protocol::ID_hash_table::ID_tree[0xB1].func_ptr_data = &control_system::set_open_loop_input_sine_amp;
    communication_protocol::ID_hash_table::ID_tree[0xB2].func_ptr_data = &control_system::set_open_loop_input_sine_freq;
    communication_protocol::ID_hash_table::ID_tree[0xB9].func_ptr_data = &control_system::set_open_loop_input_step_amp;
    communication_protocol::ID_hash_table::ID_tree[0xBA].func_ptr_data = &control_system::set_open_loop_input_step_time;
    communication_protocol::ID_hash_table::ID_tree[0xC9].func_ptr_data = &control_system::set_deadzone_compensation_dac1;
    communication_protocol::ID_hash_table::ID_tree[0xD1].func_ptr_data = &control_system::set_deadzone_compensation_dac2;
	communication_protocol::ID_hash_table::ID_tree[0x95].func_ptr_data = &controller::set_ratio;
	communication_protocol::ID_hash_table::ID_tree[0x96].func_ptr_data = &controller::set_sigma;
    communication_protocol::ID_hash_table::ID_tree[0x97].func_ptr_data = &ESO::set_st_disturbance_est_gain;
	communication_protocol::ID_hash_table::ID_tree[0xA1].func_ptr_data = &ESO::set_st_need_acc_threashold;

    return true;
}

    bool C__ID_hash_table_init(){
        return communication_protocol::ID_hash_table_init();
    }
    uint8_t C__IC_rec_buf_write(uint8_t Res, uint16_t USART_RX_STA){
        return communication_protocol::IC_rec_buf_write(Res, USART_RX_STA);
    }
    uint8_t C__PC_rec_buf_write(uint8_t Res, uint16_t USART_RX_STA){
        return communication_protocol::PC_rec_buf_write(Res, USART_RX_STA);
    }
    uint8_t C__PC_send_buf_write(const uint8_t& data){
        return communication_protocol::PC_send_buf_write(data);
    }
    bool C__set_IC_parse_flag(const int& stat){
        return communication_protocol::set_IC_parse_flag(stat);
    }
    bool C__set_PC_parse_flag(const int& stat){
        return communication_protocol::set_PC_parse_flag(stat);
    }
    bool C__get_IC_parse_flag(){
        return communication_protocol::get_IC_parse_flag();
    }
    bool C__get_PC_parse_flag(){
        return communication_protocol::get_PC_parse_flag();
    }
    float C__parse_IC_msg(){
        return communication_protocol::parse_IC_msg();
    }
    bool C__parse_PC_msg(){
        return communication_protocol::parse_PC_msg();
    }
    void C__init(){
        return communication_protocol::init();
    }

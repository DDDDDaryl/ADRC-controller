#ifndef __INFO_TO_SEND
#define __INFO_TO_SEND


#ifdef __cplusplus
extern "C"
{
#endif
	
	
struct info {
	float ref;
	float transient_profile;
	float sensor_pos;
	float err;
	float ESO_order1;
	float ESO_order2;
	float ESO_order3;
	float ctrl_sig;
};

enum info_id {
	info_id_ref = 0x81,
	info_id_transient_profile,
	info_id_output_pos,
	info_id_err,
	info_id_control_signal,
	info_id_ESO_first_order_state,
	info_id_ESO_second_order_state,
	info_id_ESO_third_order_state,
	info_id_ESO_fourth_order_state
};


union float_transmit {
	float data;
	enum info_id integer;
	char d[4];
};


#ifdef __cplusplus
}
#endif

#endif
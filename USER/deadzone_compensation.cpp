#include "deadzone_compensation.h"


template <typename T> 
T sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

float smooth_left(float ctrl_signal, float sigma) {
	return (exp(ctrl_signal / sigma)) / (exp(ctrl_signal / sigma) + exp(-ctrl_signal / sigma));
}
	
float smooth_right(float ctrl_signal, float sigma) {
	return (exp(-ctrl_signal / sigma)) / (exp(ctrl_signal / sigma) + exp(-ctrl_signal / sigma));
}

float deadzoneInverse(float ctrl_signal, float mr, float br, float ml, float bl, float sigma) {
	return (ctrl_signal + mr * br) * smooth_right(ctrl_signal, sigma)/ mr + (ctrl_signal + ml * bl) * smooth_left(ctrl_signal, sigma) / ml;
}

float delta_u(float compensated_signal, float error, float ratio) {	
	return error > 0 ? -sgn(compensated_signal) * error * ratio : sgn(compensated_signal) * error * ratio;
}


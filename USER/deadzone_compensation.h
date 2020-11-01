#ifndef __DEADZONE_COMPENSATION_H
#define __DEADZONE_COMPENSATION_H

#include <cmath>

float smooth_left(float ctrl_signal, float sigma);
float smooth_right(float ctrl_signal, float sigma);
float deadzoneInverse(float ctrl_signal, float mr, float br, float ml, float bl, float sigma);
float delta_u(float compensated_signal, float error, float ratio);
	
#endif
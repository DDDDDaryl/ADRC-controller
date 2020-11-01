#include "kalman.h"

kalman::kalman(float Q_, float R_, float init_sensor_val_) 
	: kg(0), Q(Q_), R(R_), x_pre(0), p_pre(0), p_now(0), x_pid(0),
	x_now(init_sensor_val_), x_last(init_sensor_val_), p_last(0.01)
{
	
}

float kalman::iterate(float res) {
	x_pre = x_last;
	
    p_pre = p_last + Q;

    kg = p_pre / (p_pre+ R);

    x_now = (1 - kg) * x_pre + kg * res;

    p_now = (1 - kg) * p_pre;

    p_last = p_now;

    x_last = x_now;

    return x_now;
}
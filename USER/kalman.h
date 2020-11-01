#ifndef __KALMAN_H
#define __KALMAN_H

class kalman{

private:
	
    float kg;

    float Q;

    float R;

    float x_pre;

    float p_pre;

    float p_last;

    float p_now;

    float x_now;

    float x_last;

    float x_pid;

public:
	
	kalman(float Q_, float R_, float init_sensor_val_);

	float iterate(float res);
	
};

#endif
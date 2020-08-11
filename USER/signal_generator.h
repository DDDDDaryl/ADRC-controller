#ifndef __SIGNAL_GENERATOR_H
#define __SIGNAL_GENERATOR_H

#define _MATH_DEFINES_DEFINED
#include "math.h"
#include "stdint.h"
#include "stm32f4xx.h"


#define PI										3.1415926
#define DAC_UPLOAD_FREQ 			1000		//Hz
//#define DAC_OUTPUT_DURATION		12			//s
//#define SAMPLE_POINT					DAC_UPLOAD_FREQ*DAC_OUTPUT_DURATION

u16 load_sine_output_data(void);
u8 clear_load(void);

#endif
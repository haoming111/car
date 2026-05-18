#ifndef _ENCODER_HPP_
#define _ENCODER_HPP_

#include "zf_common_headfile.hpp"

#define ENCODER_DIR_1_PATH           ZF_ENCODER_DIR_1
#define ENCODER_DIR_2_PATH           ZF_ENCODER_DIR_2

extern float Measure_Speedl,Measure_Speedr;
extern int16 speed_L,speed_R;

void get_encoder(void);

#endif

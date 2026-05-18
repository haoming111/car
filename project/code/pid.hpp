#ifndef _PID_HPP_
#define _PID_HPP_

#include "zf_common_headfile.hpp"

int32 range_protect(int32 duty, int32 min, int32 max);
void speed_pid(void);
float gyro_pid(void);
float angle_pid(void);

extern float Kp_L,Ki_L,Kd_L;
extern float Kp_R,Ki_R,Kd_R;
extern float Target_L,Target_R;
extern float out_L,out_R;
extern float Error0_L,Error1_L,delta_Error_L,delta_out_L,prev_delta_Error_L;
extern float Error0_R,Error1_R,delta_Error_R,delta_out_R,prev_delta_Error_R;


extern float Kp_g,Ki_g,Kd_g;
extern float Target_g;
extern float out_g;
extern float Error0_g,Error1_g,Integral_g;

extern float Kp_a,Ki_a,Kd_a;
extern float out_a;
extern float Error0_a,Error1_a;

#endif
#include "pid.hpp"
#include "encoder.hpp"
#include "imu660rb.hpp"
#include "run.hpp"

// /******** 限幅保护 *********/
int32 range_protect(int32 duty, int32 min, int32 max)//限幅保护
{
	if (duty >= max)    return max;

	else if (duty <= min)   return min;

	else return duty;
}

float Target_L,Actual_L,out_L;
float Kp_L=5.25,Ki_L=0.768,Kd_L=0;//    p0.768/1.0  i  0.76
float Error0_L,Error1_L,delta_Error_L,delta_out_L,prev_delta_Error_L;
float Target_R,Actual_R,out_R;
float Kp_R=5.25,Ki_R=0.768,Kd_R=0;
float Error0_R,Error1_R,delta_Error_R,delta_out_R,prev_delta_Error_R;
void speed_pid()
{
    float ActualSpeed_L = Measure_Speedl; //获取速度 
    float ActualSpeed_R = -Measure_Speedr;

    Error0_L = Target_L - ActualSpeed_L; 
    Error0_R = Target_R - ActualSpeed_R;
        // 计算误差增量
    delta_Error_L = Error0_L - Error1_L;
    delta_Error_R = Error0_R - Error1_R;

    // 计算增量输出
    delta_out_L = Kp_L * delta_Error_L + Ki_L * Error0_L + Kd_L * (delta_Error_L - prev_delta_Error_L);
    delta_out_R = Kp_R * delta_Error_R + Ki_R * Error0_R + Kd_R * (delta_Error_R - prev_delta_Error_R);

    // 更新历史误差和增量
    prev_delta_Error_L = delta_Error_L;
    prev_delta_Error_R = delta_Error_R;
    Error1_L = Error0_L;
    Error1_R = Error0_R;

    // 叠加增量到上一次输出
    out_L += delta_out_L;
    out_R += delta_out_R;
    
    //限幅
    out_L = range_protect(out_L, -950, 950);
	out_R = range_protect(out_R, -950, 950);

}

float Target_g, Actual_g, out_g;  
float Kp_g = 1.0, Ki_g = 0, Kd_g = 0;
float Error0_g, Error1_g;      // 当前误差, 上一次误差
float Integral_g = 0;          // 积分项累加器（新增）
float gyro_pid()
{
    float Actualgyro = Measure_gyro;   // 获取角速度
    Error0_g = Target_g - Actualgyro;  // 计算当前误差
    
    // 积分项累加
    Integral_g += Error0_g;
    
    // 位置式PID公式：直接计算输出值
    out_g = Kp_g * Error0_g                    // 比例项
          + Ki_g * Integral_g                  // 积分项（用累加值代替当前误差）
          + Kd_g * (Error0_g - Error1_g);      // 微分项
    
    // 更新上一次误差
    Error1_g = Error0_g;
    
    // 输出限幅
    return range_protect(out_g, -500, 500);
}


float Target_a=0,Actual_a,out_a;
float Kp_a=12,Ki_a=0,Kd_a=0; 
float Error0_a,Error1_a;
extern float angle;
float angle_pid()
{ 
    float Actualangle = angle;   
    // 当前误差
    Error0_a = Target_a - Actualangle;

    // -------- I项（积分）--------
    out_a += Ki_a * Error0_a;

    // -------- 限制积分，防止积分饱和 --------
    out_a = range_protect(out_a, -1500, 1500);

    // -------- P + I + D --------
    float pid_out =
          Kp_a * Error0_a                     // P
        + out_a                               // I
        + Kd_a * (Error0_a - Error1_a);       // D

    // 更新历史误差
    Error1_a = Error0_a;

    // 输出限幅
    return range_protect(pid_out, -2000, 2000);
}

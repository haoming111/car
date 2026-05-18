#ifndef _MOTOR_HPP_
#define _MOTOR_HPP_

#include "zf_common_headfile.hpp"

#define PWM_1_PATH        ZF_PWM_MOTOR_1
#define DIR_1_PATH        ZF_GPIO_MOTOR_1
#define PWM_2_PATH        ZF_PWM_MOTOR_2
#define DIR_2_PATH        ZF_GPIO_MOTOR_2

// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX    (drv8701e_pwm_1_info.duty_max)       
#define MOTOR2_PWM_DUTY_MAX    (drv8701e_pwm_2_info.duty_max)        
#define MAX_DUTY        (900 )   // 最大 MAX_DUTY% 占空比

void motor_init(void);
void motor(int16 left_duty,int16 right_duty);

#endif
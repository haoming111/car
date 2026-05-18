#include "motor.hpp"

extern zf_driver_pit pit_timer;

struct pwm_info drv8701e_pwm_1_info;
struct pwm_info drv8701e_pwm_2_info;


zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR);
zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR);
zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);
zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);

bool dir = true;

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    // 需要先停止定时器线程，后面才能稳定关闭电机，电调，舵机等
    pit_timer.stop();
    printf("程序异常退出，执行清理操作\n");
    // 关闭电机
    drv8701e_pwm_1.set_duty(0);   
    drv8701e_pwm_2.set_duty(0);    
}
void motor_init()
{
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);

    // 注册清理函数
    atexit(cleanup);
    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);
}

//duty = -1000 to 1000  
void motor(int16 left_duty,int16 right_duty)
{
    if(left_duty >= 0)                                                           // 正转
    {
        if(left_duty>MAX_DUTY) left_duty=MAX_DUTY;                              //限幅
        drv8701e_dir_1.set_level(1);                                      // DIR输出高电平
        drv8701e_pwm_1.set_duty(left_duty * (MOTOR1_PWM_DUTY_MAX / 1000));       // 计算占空比   
    }
    else
    {      
        if(-left_duty>MAX_DUTY) left_duty=-MAX_DUTY;
        drv8701e_dir_1.set_level(0);                                        // DIR输出低电平                                    
        drv8701e_pwm_1.set_duty(-left_duty * (MOTOR1_PWM_DUTY_MAX / 1000));       
    }
    if(right_duty >= 0)                                                           // 正转
    {      
        if(right_duty>MAX_DUTY) right_duty=MAX_DUTY;
        drv8701e_dir_2.set_level(1);                                     // DIR输出高电平
        drv8701e_pwm_2.set_duty(right_duty * (MOTOR2_PWM_DUTY_MAX / 1000));       // 计算占空比        
    }
    else
    {
        if(-right_duty>MAX_DUTY) right_duty=-MAX_DUTY;
        drv8701e_dir_2.set_level(0);                                      // DIR输出低电平
        drv8701e_pwm_2.set_duty(-right_duty * (MOTOR2_PWM_DUTY_MAX / 1000));            
    }

}
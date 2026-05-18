#include "interrupt.hpp"
#include "encoder.hpp"
#include "key.hpp"
#include "imu660rb.hpp"
#include "pid.hpp"
#include "motor.hpp"

int16 car_speed = 0;  //最低速40
extern int Lmidnum2,Rmidnum2;
extern float angle;
extern int zebra_use; 
void pit_callback(void)
{
    static uint8 time_i=0;
    static uint8 time_a=0;
    if(time_i>0)
    {
        key_scanf();
        if(Lmidnum2==0&&Rmidnum2==0)
        {
            angle=0;
        } 
        if(zebra_use>1)
        {
            time_a++;
            if(time_a>10)
            {
                car_speed-=10;
                if(car_speed<10)
                {
                    car_speed=0; 
                }
                time_a=0;
            }
        }
        Target_g=angle_pid();
        get_raw();  //Target_L=30 Target_R=-30   540
        if(car_star)
        {  
            float gyro_add=gyro_pid();
            // extern int shizi;
            // if (shizi>3)
            // {
            //     // 打印诊断信息并清零校正量
            //     printf("[DIAG] shizi=%d Measure_gyro=%f Target_g=%f gyro_add(before)=%f\n", shizi, Measure_gyro, Target_g, gyro_add);
            // }
            Target_L= car_speed -gyro_add;
            Target_R= car_speed +gyro_add;
            // 输出用于诊断的目标速度

            // printf("[DIAG] gyro_add=%f angle=%f Target_g=%f\n", gyro_add, angle, Target_g);
        }
        time_i=0;
    }     
    if(car_star)
    {          
        get_encoder();
        speed_pid();    //Target_L=40 Target_R=40
        motor(out_L, -out_R);
    }
    time_i++;
}

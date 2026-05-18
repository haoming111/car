#include "key.hpp"
#include "pid.hpp"
#define BEEP_PATH        ZF_GPIO_BEEP
zf_driver_gpio  beep(BEEP_PATH, O_RDWR);//蜂鸣器

zf_driver_gpio  key_1(KEY_1_PATH, O_RDWR);
zf_driver_gpio  key_2(KEY_2_PATH, O_RDWR);
zf_driver_gpio  key_3(KEY_3_PATH, O_RDWR);
zf_driver_gpio  key_4(KEY_4_PATH, O_RDWR);

// 全局标志位
volatile bool key_scan_flag = false;
struct keys key[4]={0,0,0,0};

void key_scanf()
{       
    key[0].key_sta = key_1.get_level();
    key[1].key_sta = key_2.get_level();
    key[2].key_sta = key_3.get_level();
    key[3].key_sta = key_4.get_level();

    for(int i = 0; i < 4; i++) 
    {
        switch(key[i].key_jude)
        {
            case 0: // 等待按下
                if(key[i].key_sta == 0)  // 检测到按下
                {
                    key[i].key_jude = 1;  // 进入消抖状态
                }
                key[i].key_time = 0;
                break;
                
            case 1: // 消抖确认
                if(key[i].key_sta == 0)
                {
                    key[i].key_jude = 2;  // 确认按下
                }
                else
                {
                    key[i].key_jude = 0;  // 抖动，返回空闲
                }
                break;
                
            case 2: // 按下保持
                if(key[i].key_sta == 1)  // 按键释放
                {
                    if(key[i].key_time < 200)  // 短按
                    {
                        key[i].sin_flag = 1;
                    }
                    // 超过200ms为长按，已在长按时处理
                    key[i].key_jude = 0;  // 返回空闲
                    key[i].key_time = 0;
                }
                else  // 保持按下
                {
                    key[i].key_time++;
                    if(key[i].key_time == 200)  // 长按触发
                    {
                        key[i].long_flag = 1;
                        // 添加状态3防止重复触发
                        key[i].key_jude = 3;
                    }
                }
                break;
                
            // 可选：增加状态3防止长按重复触发
            case 3: // 长按已触发，等待释放
                if(key[i].key_sta == 1)  // 释放
                {
                    key[i].key_jude = 0;
                    key[i].key_time = 0;
                }
                break;
        }
    }

}

extern zf_device_ips200 ips200;
extern int16 car_speed ;  //最低速40
bool car_star=0;
extern int qian;
void  key_event_handler()  // 处理按键事件 
{
    if(key[0].sin_flag==1)
    {                            
        car_speed+=20 ;
        ips200.show_int(200,16*0,car_speed, 3);
        key[0].sin_flag=0;
    }
    if(key[0].long_flag==1)
    {		       
        car_speed-=10 ;
        ips200.show_int(200,16*0,car_speed, 3);
        key[0].long_flag=0;		
    }  

    if(key[1].sin_flag==1)
    {                     
        
        qian+=1;
        ips200.show_int(200,16*1,qian,2);   
        key[1].sin_flag=0;
    }
    if(key[1].long_flag==1)
    {		       
        qian-=1;
        ips200.show_int(200,16*1,qian,2);   
        key[1].long_flag=0;		
    }  

    if(key[2].sin_flag==1)
    {                     
        Kd_a+=0.008;
        Error0_a=0;
        Error1_a=0;
        out_a=0;
        ips200.show_float(200,16*2,Kd_a,2, 4);  
        key[2].sin_flag=0;
    }
    if(key[2].long_flag==1)
    {		       
        Kd_a-=0.008;
        Error0_a=0;
        Error1_a=0;
        out_a=0;
        ips200.show_float(200,16*2,Kd_a,2, 4);  
        key[2].long_flag=0;		
    }  

    if(key[3].sin_flag==1)
    {                     
        car_star=!car_star;
        key[3].sin_flag=0;
    }
    if(key[3].long_flag==1)
    {		       

        key[3].long_flag=0;		
    }  
}
#include "zf_common_headfile.hpp"
#include "tfilte.hpp"
#include "key.hpp"
#include "interrupt.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "imu660rb.hpp"
#include "camera_send.hpp"
#include "rot.hpp"
#include "image.hpp"
#include "run.hpp"
#include "ring.hpp"
#include "pid.hpp"
#include "zebra.hpp"
#include "red.hpp"
#include "tof.hpp"

zf_driver_pit pit_timer;
zf_device_ips200 ips200;

float angle;
int red_num=0;
int red_state=0;
int red_juli=0;
int red_qian;

extern uint8 yuzhi;
extern int Lout_count,Rout_count; 
extern int Lmidnum2,Rmidnum2,Rmidnum,Lmidnum;
extern float Luse_edge[100][2],Ruse_edge[100][2];     //左边界结构体
extern float Lout_MID[50][2],Rout_MID[50][2];
extern float Lin_MID[50][2],Rin_MID[50][2];

extern int xunxian_dir;
extern int qianzhan;
extern int qian;
extern bool car_star;
extern int souxian;
int main(int, char**) 
{
    motor_init();//电机初始化
    imu660rb_init();//陀螺仪初始化
    // tof_init();
    ips200.init(FB_PATH);//屏幕初始化
    camera_init();//摄像头初始化
    // camera_send_init();//图传初始化
    pit_timer.init_ms(5, pit_callback);//定时器初始化   创建一个定时器5ms周期，回调函数为pit_callback
    while(1)
    {
        Image_Get();
        key_event_handler();
        yuzhi=otsuThreshold(image_use[0]);
        // int16 tof_distance=get_tof();
        
        // seekfree_assistant_camera_send();//发送图像到上位机    
        // printf("%f,%f\n", Measure_Speedl,Measure_Speedr);//转速对比滤波转速
        // motor(300, -300);
        // printf("%f,%f\n", Target_g,Measure_gyro);//pid目标与实际
        if(car_star==0)
        {           
            // show_image(0);// 显示图像到屏幕上
            // for (int y = 0; y < UVC_HEIGHT; y++)//逆透视
            // {
            //     for (int x = 0; x < UVC_WIDTH; x++)
            //     {
            //         // 1. 逆透视：显示坐标 → 原图坐标
            //         int src_x = (int)XShowRotimg(x, y);
            //         int src_y = (int)YShowRotimg(x, y);
            //         // 2. 判断是否有效区域
            //         if(src_x < 0 || src_x >= UVC_WIDTH || src_y < 0 || src_y >= UVC_HEIGHT)
            //         {
            //             continue;   //无效区域直接不画
            //         }
            //         // 3. 取原图像素
            //         if (image_use[src_y][src_x] > yuzhi)
            //         {
            //             ips200.draw_point(x, y + 16 * 9, RGB565_WHITE);
            //         }
            //         else
            //         {
            //             ips200.draw_point(x, y + 16 * 9, RGB565_BLACK);
            //         }
            //     }
            // }

             for (int y = 0; y < UVC_HEIGHT; y++)//逆透视白屏
            {
                for (int x = 0; x < UVC_WIDTH; x++)
                {
                    ips200.draw_point(x, y + 16 * 9, RGB565_WHITE);
                }
            }

             for (int y = 0; y < UVC_HEIGHT; y++)   //二值化原图
            {
                for (int x = 0; x < UVC_WIDTH; x++)
                {
                    if (image_use[y][x] > yuzhi)
                    {
                        ips200.draw_point(x, y, RGB565_WHITE);
                    }
                    else
                    {
                        ips200.draw_point(x, y, RGB565_BLACK);
                    }
                }
            }
            // for (int y = 0; y < UVC_HEIGHT; y++)      //红色二值化图
            // {
            //     for (int x = 0; x < UVC_WIDTH; x++)
            //     {
            //         uint16_t pixel = rgb_image[y * UVC_WIDTH + x];

            //         // ===== RGB565拆分 =====
            //         uint8_t r = (pixel >> 11) & 0x1F;
            //         uint8_t g = (pixel >> 5)  & 0x3F;
            //         uint8_t b = pixel & 0x1F;

            //         // 放大到0~255
            //         r <<= 3;
            //         g <<= 2;
            //         b <<= 3;

            //         // ===== 红色判断条件（你可以调这里）=====
            //         if (r > 100 && (r - g) > 40 && (r - b) > 40)
            //         {
            //             ips200.draw_point(x, y, RGB565_WHITE); // 判定为红色
            //         }
            //         else
            //         {
            //             ips200.draw_point(x, y, RGB565_BLACK);
            //         }
            //     }
            // }
            ips200.draw_line(        
            (uint16)50,
            (uint16)70,
            (uint16)110,
            (uint16)72,
            RGB565_RED
            );
            ips200.draw_line(       
            (uint16)60,
            (uint16)86,
            (uint16)100,
            (uint16)88,
            RGB565_RED
            );
        }

        switch (red_state)
        {
            case 0:
                    if(is_red_region(rgb_image,UVC_WIDTH,UVC_HEIGHT, 50, 110 ,  70, 72))
                    {
                        red_state=1;
                        printf("r\n");
                    }
            break;
        
            case 1: 
                    // if(is_red_region(rgb_image,UVC_WIDTH,UVC_HEIGHT, 60, 100 ,  86, 88))
                    // {
                    //     red_num++;
                    // }
                    // else
                    // {
                        red_juli+=Measure_Speedl-Measure_Speedr;
                    // }
                    
                    // if(red_num>3)
                    // {
                    //     red_state=2;
                    //     souxian=55;
                    //     red_juli=0;
                    //     red_num=0;
                    //     red_qian=qian;
                    //     qian=3;
                    // }
                    // else 
                    if (red_juli>8000)
                    {
                        red_state=0;
                        red_juli=0;
                        red_num=0;
                        printf("e\n");
                    }
                break;

                case 2:
                    if(is_red_region(rgb_image,UVC_WIDTH,UVC_HEIGHT, 60, 100 ,  86, 88)==0)
                    {
                        red_num++;
                    }
                    if(red_num>6)
                    {
                        red_juli+=Measure_Speedl-Measure_Speedr;
                        // ips200.show_int(50,16*18,red_juli,4); 
                        if(red_juli>1000&&red_juli<2000)
                        {
                            souxian=40;
                            qian=4;
                        }
                        if (red_juli>3000)
                        {
                            printf("d\n");
                            red_juli=0;
                            red_state=0;
                            souxian=30;
                            red_num=0;
                            qian=red_qian;
                        }
                    }
            break;        
        }
         
        // ips200.show_int(50,16*19,tof_distance,4); 

        ifzebra();
        if(ring_ctrl.kind == no_ring &&shizi==0 )  //80  13    10
        {
            qianzhan=qian;
            search_bianxian();
        }
        if(ring_ctrl.kind != no_ring)
        {
            qianzhan=qian-1;
        }
        if_luzhang();
        if_shizi();
        Ring_Process();

        zhongxianhecheng();
        
        xunxain_celue();

        angle = get_angle();   

        if(car_star==0)
        {
            if(ring_ctrl.kind == no_ring)
            {
                //左边界（红色）
                for (int i = 0; i < Lout_count; i++)
                {
                    int x = (int)Luse_edge[i][0];
                    int y = (int)Luse_edge[i][1];
                    // 判断是否有效区域
                    if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                    {
                        continue;   //无效区域直接不画
                    }
                    if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
                    {
                        ips200.draw_point(x, y+16*9, RGB565_RED);
                        ips200.draw_point(x+1, y+16*9, RGB565_RED);
                    }
                }
                    // 右边界（蓝色）
                for (int i = 0; i < Rout_count; i++)
                {
                    int x = (int)Ruse_edge[i][0];
                    int y = (int)Ruse_edge[i][1];
                    if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                    {
                        continue;   //无效区域直接不画
                    }
                    if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
                    {
                        ips200.draw_point(x, y+16*9, RGB565_BLUE);
                        ips200.draw_point(x+1, y+16*9, RGB565_BLUE);
                    }        
                }
            }
            //左中线
            for (int i = 0; i < Lmidnum2; i++)
            {
                int x = (int)Lout_MID[i][0];
                int y = (int)Lout_MID[i][1];
                 if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                {
                    continue;   //无效区域直接不画
                }
                if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT)
                {
                    ips200.draw_point(x, y+16*9, RGB565_RED);
                    ips200.draw_point(x+1, y+16*9, RGB565_RED);
                }               
            }

            // 右中线
            for (int i = 0; i < Rmidnum2; i++)
            {
                int x = (int)Rout_MID[i][0];
                int y = (int)Rout_MID[i][1];
                 if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                {
                    continue;   //无效区域直接不画
                }
                if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT)
                {
                    ips200.draw_point(x, y+16*9, RGB565_BLUE);
                    ips200.draw_point(x+1, y+16*9, RGB565_BLUE);
                }
            }

            int screen_offset = 16 * 9;  // 144
            if(xunxian_dir==-1)
            {
                if (Lmidnum2 > 0) 
                {
                    int line_x1=0;int line_y1=0;
                    if(Lmidnum2>qianzhan)
                    {
                        line_x1 = (int)Lout_MID[qianzhan][0];
                        line_y1 = (int)Lout_MID[qianzhan][1];
                    }
                    else line_x1 = (int)Lout_MID[Lmidnum2 - 1][0];
                    if (line_x1 >= 0 && line_x1 < UVC_WIDTH) {
                        ips200.draw_line(
                            (uint16)line_x1,
                            (uint16)(line_y1 + screen_offset),//起始行
                            (uint16)line_x1,
                            (uint16)((UVC_HEIGHT - 1) + screen_offset),
                            RGB565_BLACK
                        );
                    }
                }
            }
            if(xunxian_dir==1)
            {
                if (Rmidnum > 0) 
                {
                    int line_x2=0;int line_y2=0;
                    if (Rmidnum2>qianzhan)
                    {
                        line_x2 = (int)Rout_MID[qianzhan][0];
                        line_y2 = (int)Rout_MID[qianzhan][1];
                    }
                    else line_x2 = (int)Rout_MID[Rmidnum - 1][0];
                    if (line_x2 >= 0 && line_x2 < UVC_WIDTH) 
                    {
                        ips200.draw_line(
                            (uint16)line_x2,
                            (uint16)(line_y2 + screen_offset),
                            (uint16)line_x2,
                            (uint16)((UVC_HEIGHT - 1) + screen_offset),
                            RGB565_BLACK
                        );
                    }
                }
            }      
            
            ips200.draw_line(       //搜线起始行
                        (uint16)0,
                        (uint16)UVC_HEIGHT-souxian,//搜线起始行
                        (uint16)UVC_WIDTH,
                        (uint16)UVC_HEIGHT-souxian,
                        RGB565_RED
                    );
            // ips200.draw_line(   //中线
            //         (uint16)80,  //中线
            //         (uint16)(0 + screen_offset),
            //         (uint16)80,
            //         (uint16)((UVC_HEIGHT - 1) + screen_offset),
            //         RGB565_BLACK
            //     );
                
            //         float dx=Lout_MID[Lmidnum2-1][0]-UVC_WIDTH/2,dy=UVC_HEIGHT+20-Lout_MID[Lmidnum2-1][1];//5   +16 18
            //         float angle=1*atan2f(dx, dy) * 180 / PI;  

                    // ips200.show_int(0,16*17,xunxian_dir,2); //左右寻线标志位
                    // ips200.show_float(50,16*17,get_angle(), 3,3);//误差角度
            //         ips200.show_float(50,16*17,dx, 3,3);

            //         ips200.show_int(0,16*19,Lmidnum2, 3);//zhongxian 车的中心列位置是95
            //         ips200.show_float(50,16*19,Lout_MID[18][0], 3,3);


                    // printf("zf_encoder_left = %d.\r\n", speed_L);//20  占空比  编码器 90
                    // printf("zf_encoder_right = %d.\r\n", speed_R);//30 占空比  编码器 158

                    // printf("imu_gyro_x = %d\r\n", imu_gyro_x);
                    // printf("imu_gyro_y = %d\r\n", imu_gyro_y);
                    // printf("imu_gyro_z = %d\r\n", imu_gyro_z);
                    // printf("Measure_gyro = %f\r\n", Measure_gyro);
                    // ips200.show_int(0,0,imu_gyro_x,4);
                    // ips200.show_int(0,1*16,imu_gyro_y,4);
                    // ips200.show_int(2,2*16,imu_gyro_z,4);
                    // ips200.show_float(0,3*16,Measure_gyro,4,4);
        }
         
            // ips200.show_float(50,16*17,Target_g, 3,3);
    }

    return 0; 
}
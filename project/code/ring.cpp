#include "ring.hpp"
#include "run.hpp"
#include "math.h"
#include "image.hpp"
#include "key.hpp"
#include "encoder.hpp"
#include "rot.hpp"
#include "camera_send.hpp"
#include "imu660rb.hpp"
#include "zebra.hpp"

extern zf_device_ips200 ips200;

RingState ring_ctrl = {
    .state = Ring_state1,
    .kind = no_ring,
    .phase_counter = 0,
    .sum_zangle = 0,
    .time_counter = 0,
    .flag = 0
};
int  n_count; //当前边个数
int  w_count;  //对边
int16 width_base =90; //未逆透视赛道宽度90
float n_edge[100][2],w_edge[100][2];//当前边/对边 边界数组
float yanshen[100][2];
uint8 yanshen_count;
bool out_flag=0;
bool int_flag=0;

void InitRingContext() 
{
    ring_ctrl.last_high = 0;
    ring_ctrl.state = Ring_state1;
    ring_ctrl.phase_counter = 0;
    ring_ctrl.sum_zangle = 0;
    ring_ctrl.time_counter = 0;
    ring_ctrl.flag = 0;
}

void Ring_switch(void)
{
    if(ring_ctrl.phase_counter >= 3)
    {
        ring_ctrl.state = static_cast<ring_state>(static_cast<int>(ring_ctrl.state) + 1);
        ring_ctrl.phase_counter = 0; 
        Ring_Process();
    }
}

extern bool BU_flag;
extern int red_state;
void Ring_Process(void)
{
    // ips200.show_int(0,16*17,ring_ctrl.kind,2);
    // ips200.show_int(50,16*17,ring_ctrl.state,2);
    // ips200.show_int(100,16*17,lost_l,2);
    // ips200.show_int(150,16*17,lost_r,2);
    // ips200.show_int(200,16*17,ring_ctrl.sum_zangle,4);
    if(ring_ctrl.kind == no_ring )
    {
        if(ifzhi_L==1&&ang_numR>0&&ang_numR<5&&shizi==0 && zebra_state==0 && luzhang==0&&red_state==0 )
        {
                ring_ctrl.kind = right_ring;
        }
        else if(ifzhi_R==1&&ang_numL>0&&ang_numL<5 &&shizi==0 && zebra_state==0 && luzhang==0&&red_state==0)
        {
                ring_ctrl.kind = left_ring;
        }
        else 
        {
            ring_ctrl.flag = 0;
        }
        if(ring_ctrl.kind != no_ring){
            InitRingContext();
        }
    }
    else
    {
            get_highest();
            lost_l=0;
            lost_r=0;
            get_BLY(L_edge,&L_count,R_edge,&R_count,30,0,0);
            if(L_count< 5)
            {
                lost_l=1;
            }
            if(R_count < 5)
            {
                lost_r=1;
            }
            //   for (int i = 0; i < L_count; i++)
            // {
            //     int x = (int)L_edge[i][0];
            //     int y = (int)L_edge[i][1];
            //     // 判断是否有效区域
            //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
            //     {
            //         continue;   //无效区域直接不画
            //     }
            //     if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT)
            //     {
            //         ips200.draw_point(x, y, RGB565_RED);
            //     }
            // }

            // // 右边界（蓝色）
            // for (int i = 0; i < R_count; i++)
            // {
            //     int x = (int)R_edge[i][0];
            //     int y = (int)R_edge[i][1];
            //         if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
            //     {
            //         continue;   //无效区域直接不画
            //     }
            //     if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT)
            //     {
            //         ips200.draw_point(x, y, RGB565_BLUE);
            //     }                   
            // }

        if(ring_ctrl.kind == left_ring)
        {
            n_count=L_count;
            memcpy(n_edge, L_edge, sizeof(L_edge));
            w_count=R_count;
            memcpy(w_edge, R_edge, sizeof(R_edge));
        }
        else if(ring_ctrl.kind == right_ring)
        {
            n_count=R_count;
            memcpy(n_edge, R_edge, sizeof(R_edge));
            w_count=L_count;
            memcpy(w_edge, L_edge, sizeof(L_edge));
        }

        if(ring_ctrl.state >= Ring_state3)
            ring_ctrl.sum_zangle += Measure_gyro ;

        float dx = 0;
        float dy = 0;
        float distance = 0;
        switch(ring_ctrl.state)
        {
            case Ring_state1:
            beep.set_level(1);
                if((ring_ctrl.kind==left_ring && lost_l )||(ring_ctrl.kind==right_ring && lost_r) )//丢线
                    ring_ctrl.phase_counter++;
                else
                {
                    if(ring_ctrl.phase_counter > 0)
                        ring_ctrl.phase_counter--;
                }
                if(ring_ctrl.phase_counter > PHASE_COUNT_THRESHOLD)
                {
                    ring_ctrl.state = Ring_state2;
                    ring_ctrl.phase_counter = 0;
                }
                //**********************************边线逆透视
                touxian(L_edge,&L_count);
                touxian(R_edge,&R_count);

                //**********************************边线滤波
                blur_points(L_edge,Lout_edge,L_count,3);
                blur_points(R_edge,Rout_edge,R_count,3);

                //**********************************边线重采样
                Lout_count=L_count*2;
                Rout_count=R_count*2;
                resample_points2(Lout_edge,L_count,Luse_edge,&Lout_count,3);
                resample_points2(Rout_edge,R_count,Ruse_edge,&Rout_count,3);

                //**********************************找拐点
                guai_mum(Luse_edge,Lout_count,ang_L,&ang_numL,&ifzhi_L);
                guai_mum(Ruse_edge,Rout_count,ang_R,&ang_numR,&ifzhi_R);

                    if (ang_numL>0)
                    {
                        Lout_count=ang_numL;

                        // int x = (int)Luse_edge[ang_numL][0];
                        // int y = (int)Luse_edge[ang_numL][1];
                        // if (x >= 3 && x < UVC_WIDTH-3 && y >= 3 && y < UVC_HEIGHT-3)
                        // {
                        //     ips200.draw_point(x-1, y+16*9, RGB565_GREEN);
                        //     ips200.draw_point(x, y+16*9, RGB565_GREEN);
                        //     ips200.draw_point(x+1, y+16*9, RGB565_GREEN);
                        //     ips200.draw_point(x, y+16*9-1, RGB565_GREEN);
                        //     ips200.draw_point(x, y+16*9+1, RGB565_GREEN);
                        // }
                    }
                    if (ang_numR>0)
                    {
                        Rout_count=ang_numR;

                        // int a = (int)Ruse_edge[ang_numR][0];
                        // int b = (int)Ruse_edge[ang_numR][1];
                        // if (a >= 3 && a < UVC_WIDTH-3 && b >= 3 && b < UVC_HEIGHT-3)
                        // {
                        //     ips200.draw_point(a-1, b+16*9, RGB565_GREEN);
                        //     ips200.draw_point(a, b+16*9, RGB565_GREEN);
                        //     ips200.draw_point(a+1, b+16*9, RGB565_GREEN);
                        //     ips200.draw_point(a, b+16*9-1, RGB565_GREEN);
                        //     ips200.draw_point(a, b+16*9+1, RGB565_GREEN);
                        // }
                    }
            // for (int i = 0; i < Lout_count; i++)
            // {
            //     int x = (int)Luse_edge[i][0];
            //     int y = (int)Luse_edge[i][1];
            //     // 判断是否有效区域
            //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
            //     {
            //         continue;   //无效区域直接不画
            //     }
            //     if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT)
            //     {
            //         ips200.draw_point(x, y+16*9, RGB565_RED);
            //     }
            // }
            //     // 右边界（蓝色）
            // for (int i = 0; i < Rout_count; i++)
            // {
            //     int x = (int)Ruse_edge[i][0];
            //     int y = (int)Ruse_edge[i][1];
            //      if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
            //     {
            //         continue;   //无效区域直接不画
            //     }
            //     if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT)
            //     {
            //         ips200.draw_point(x, y+16*9, RGB565_BLUE);
            //     }        
            // }
                break;

            case Ring_state2:
            beep.set_level(0);
                if(lost_l==0&&lost_r==0) 
                ring_ctrl.phase_counter++;
                if(ring_ctrl.phase_counter >= PHASE_COUNT_THRESHOLD)
                {
                    ring_ctrl.state = Ring_state3;
                    ring_ctrl.phase_counter = 0;
                }
                //**********************************边线逆透视
                touxian(L_edge,&L_count);
                touxian(R_edge,&R_count);

                //**********************************边线滤波
                blur_points(L_edge,Lout_edge,L_count,3);
                blur_points(R_edge,Rout_edge,R_count,3);

                //**********************************边线重采样
                Lout_count=L_count*2;
                Rout_count=R_count*2;
                resample_points2(Lout_edge,L_count,Luse_edge,&Lout_count,3);
                resample_points2(Rout_edge,R_count,Ruse_edge,&Rout_count,3);

                //**********************************找拐点
                guai_mum(Luse_edge,Lout_count,ang_L,&ang_numL,&ifzhi_L);
                guai_mum(Ruse_edge,Rout_count,ang_R,&ang_numR,&ifzhi_R);

                    if (ang_numL>0)
                    {
                        Lout_count=ang_numL;

                        // int x = (int)Luse_edge[ang_numL][0];
                        // int y = (int)Luse_edge[ang_numL][1];
                        // if (x >= 3 && x < UVC_WIDTH-3 && y >= 3 && y < UVC_HEIGHT-3)
                        // {
                        //     ips200.draw_point(x-1, y+16*9, RGB565_GREEN);
                        //     ips200.draw_point(x, y+16*9, RGB565_GREEN);
                        //     ips200.draw_point(x+1, y+16*9, RGB565_GREEN);
                        //     ips200.draw_point(x, y+16*9-1, RGB565_GREEN);
                        //     ips200.draw_point(x, y+16*9+1, RGB565_GREEN);
                        // }
                    }
                    if (ang_numR>0)
                    {
                        Rout_count=ang_numR;

                        // int a = (int)Ruse_edge[ang_numR][0];
                        // int b = (int)Ruse_edge[ang_numR][1];
                        // if (a >= 3 && a < UVC_WIDTH-3 && b >= 3 && b < UVC_HEIGHT-3)
                        // {
                        //     ips200.draw_point(a-1, b+16*9, RGB565_GREEN);
                        //     ips200.draw_point(a, b+16*9, RGB565_GREEN);
                        //     ips200.draw_point(a+1, b+16*9, RGB565_GREEN);
                        //     ips200.draw_point(a, b+16*9-1, RGB565_GREEN);
                        //     ips200.draw_point(a, b+16*9+1, RGB565_GREEN);
                        // }
                    }
            // for (int i = 0; i < Lout_count; i++)
            // {
            //     int x = (int)Luse_edge[i][0];
            //     int y = (int)Luse_edge[i][1];
            //     // 判断是否有效区域
            //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
            //     {
            //         continue;   //无效区域直接不画
            //     }
            //     if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT)
            //     {
            //         ips200.draw_point(x, y+16*9, RGB565_RED);
            //     }
            // }
            //     // 右边界（蓝色）
            // for (int i = 0; i < Rout_count; i++)
            // {
            //     int x = (int)Ruse_edge[i][0];
            //     int y = (int)Ruse_edge[i][1];
            //      if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
            //     {
            //         continue;   //无效区域直接不画
            //     }
            //     if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT)
            //     {
            //         ips200.draw_point(x, y+16*9, RGB565_BLUE);
            //     }        
            // }
                break;

            case Ring_state3:
            case Ring_state4:
            beep.set_level(1);
                Ring_evolve();

                if(BU_star_flag==0)
                {
                    //**********************************边线逆透视
                    touxian(L_edge,&L_count);
                    touxian(R_edge,&R_count);

                    //**********************************边线滤波
                    blur_points(L_edge,Lout_edge,L_count,3);
                    blur_points(R_edge,Rout_edge,R_count,3);

                    //**********************************边线重采样
                    Lout_count=L_count*2;
                    Rout_count=R_count*2;
                    resample_points2(Lout_edge,L_count,Luse_edge,&Lout_count,3);
                    resample_points2(Rout_edge,R_count,Ruse_edge,&Rout_count,3);

                    //**********************************找拐点
                    guai_mum(Luse_edge,Lout_count,ang_L,&ang_numL,&ifzhi_L);
                    guai_mum(Ruse_edge,Rout_count,ang_R,&ang_numR,&ifzhi_R);

                        if (ang_numL>0)
                        {
                            Lout_count=ang_numL;

                            // int x = (int)Luse_edge[ang_numL][0];
                            // int y = (int)Luse_edge[ang_numL][1];
                            // if (x >= 3 && x < UVC_WIDTH-3 && y >= 3 && y < UVC_HEIGHT-3)
                            // {
                            //     ips200.draw_point(x-1, y+16*9, RGB565_GREEN);
                            //     ips200.draw_point(x, y+16*9, RGB565_GREEN);
                            //     ips200.draw_point(x+1, y+16*9, RGB565_GREEN);
                            //     ips200.draw_point(x, y+16*9-1, RGB565_GREEN);
                            //     ips200.draw_point(x, y+16*9+1, RGB565_GREEN);
                            // }
                        }
                        if (ang_numR>0)
                        {
                            Rout_count=ang_numR;

                            // int a = (int)Ruse_edge[ang_numR][0];
                            // int b = (int)Ruse_edge[ang_numR][1];
                            // if (a >= 3 && a < UVC_WIDTH-3 && b >= 3 && b < UVC_HEIGHT-3)
                            // {
                            //     ips200.draw_point(a-1, b+16*9, RGB565_GREEN);
                            //     ips200.draw_point(a, b+16*9, RGB565_GREEN);
                            //     ips200.draw_point(a+1, b+16*9, RGB565_GREEN);
                            //     ips200.draw_point(a, b+16*9-1, RGB565_GREEN);
                            //     ips200.draw_point(a, b+16*9+1, RGB565_GREEN);
                            // }
                        }
                        // for (int i = 0; i < Lout_count; i++)
                        // {
                        //     int x = (int)Luse_edge[i][0];
                        //     int y = (int)Luse_edge[i][1];
                        //     // 判断是否有效区域
                        //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                        //     {
                        //         continue;   //无效区域直接不画
                        //     }
                        //     if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
                        //     {
                        //         ips200.draw_point(x, y+16*9, RGB565_RED);
                        //         ips200.draw_point(x+1, y+16*9, RGB565_RED);
                        //     }
                        //     // if(x<5) 
                        // }
                        //     // 右边界（蓝色）
                        // for (int i = 0; i < Rout_count; i++)
                        // {
                        //     int x = (int)Ruse_edge[i][0];
                        //     int y = (int)Ruse_edge[i][1];
                        //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                        //     {
                        //         continue;   //无效区域直接不画
                        //     }
                        //     if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
                        //     {
                        //         ips200.draw_point(x, y+16*9, RGB565_BLUE);
                        //         ips200.draw_point(x+1, y+16*9, RGB565_BLUE);
                        //     }        
                        // }
                    }
                break;

            case Ring_state5:
            beep.set_level(0);
                    touxian(n_edge,&n_count);
                    touxian(w_edge,&w_count);
                    blur_points(n_edge,Lout_edge,n_count,3);
                    blur_points(w_edge,Rout_edge,w_count,3);
                    L_count=n_count*2;
                    R_count=w_count*2;
                    resample_points2(Lout_edge,n_count,n_edge,&L_count,3);
                    resample_points2(Rout_edge,w_count,w_edge,&R_count,3);
                    guai_mum(n_edge,L_count,ang_L,&ang_numL,&ifzhi_L);
                    guai_mum(w_edge,R_count,ang_R,&ang_numR,&ifzhi_R);
                    //  //左边界（红色）
                    // for (int i = 0; i < L_count; i++)
                    // {
                    //     int x = (int)n_edge[i][0];
                    //     int y = (int)n_edge[i][1];
                    //     // 判断是否有效区域
                    //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                    //     {
                    //         continue;   //无效区域直接不画
                    //     }
                    //     if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
                    //     {
                    //         ips200.draw_point(x, y+16*9, RGB565_RED);
                    //         ips200.draw_point(x+1, y+16*9, RGB565_RED);
                    //     }
                    // }

                    // // 右边界（蓝色）
                    // for (int i = 0; i < R_count; i++)
                    // {
                    //     int x = (int)w_edge[i][0];
                    //     int y = (int)w_edge[i][1];
                    //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                    //     {
                    //         continue;   //无效区域直接不画
                    //     }
                    //     if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
                    //     {
                    //         ips200.draw_point(x, y+16*9, RGB565_BLUE);
                    //         ips200.draw_point(x+1, y+16*9, RGB565_BLUE);
                    //     }
                            
                    // }
                        if (ang_numR>0)
                        {
                            // int a = (int)w_edge[ang_numR][0];
                            // int b = (int)w_edge[ang_numR][1];
                            // if (a >= 3 && a < UVC_WIDTH-3 && b >= 3 && b < UVC_HEIGHT-3)
                            // {
                            //     ips200.draw_point(a-1, b+16*9, RGB565_GREEN);
                            //     ips200.draw_point(a, b+16*9, RGB565_GREEN);
                            //     ips200.draw_point(a+1, b+16*9, RGB565_GREEN);
                            //     ips200.draw_point(a, b+16*9-1, RGB565_GREEN);
                            //     ips200.draw_point(a, b+16*9+1, RGB565_GREEN);
                            // }
                        }
                    Ring_Out();
                    if(BU_star_flag==0)
                    {
                        if(ring_ctrl.kind == left_ring)
                        {
                            memcpy(Luse_edge, n_edge, sizeof(n_edge));
                            Lout_count=L_count;
                            memcpy(Ruse_edge, w_edge, sizeof(w_edge));
                            Rout_count=R_count;
                        }
                        else if(ring_ctrl.kind == right_ring)
                        {
                            memcpy(Ruse_edge, n_edge, sizeof(n_edge));
                            Lout_count=R_count;
                            memcpy(Luse_edge, w_edge, sizeof(w_edge));
                            Rout_count=L_count;
                        }
                        // //左边界（红色）
                        // for (int i = 0; i < L_count; i++)
                        // {
                        //     int x = (int)Luse_edge[i][0];
                        //     int y = (int)Luse_edge[i][1];
                        //     // 判断是否有效区域
                        //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                        //     {
                        //         continue;   //无效区域直接不画
                        //     }
                        //     if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
                        //     {
                        //         ips200.draw_point(x, y+16*9, RGB565_RED);
                        //         ips200.draw_point(x+1, y+16*9, RGB565_RED);
                        //     }
                        // }
                        // // 右边界（蓝色）
                        // for (int i = 0; i < R_count; i++)
                        // {
                        //     int x = (int)Ruse_edge[i][0];
                        //     int y = (int)Ruse_edge[i][1];
                        //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                        //     {
                        //         continue;   //无效区域直接不画
                        //     }
                        //     if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
                        //     {
                        //         ips200.draw_point(x, y+16*9, RGB565_BLUE);
                        //         ips200.draw_point(x+1, y+16*9, RGB565_BLUE);
                        //     }                          
                        // }
                    }
                    if(out_flag==1)
                    {       
                        // // 如果高度达到并且累计角度足够大，则增加阶段计数
                        // if(w_edge[ang_numR][1] > Mini_high - 10 && ring_ctrl.sum_zangle > 200 && ang_numR > 3)
                        // {
                        //     ring_ctrl.phase_counter++;
                        // }
                        // 如果边界丢失并且角度累计足够，说明接近出环          
                        if(ring_ctrl.kind == left_ring && lost_r&& fabs(ring_ctrl.sum_zangle) > 200)
                        {
                            ring_ctrl.phase_counter++;
                            // 将边界截断到corner位置
                            yanshen_count = ang_numR;
                        }
                        
                        else if(ring_ctrl.kind == right_ring && lost_l && fabs(ring_ctrl.sum_zangle) > 200)
                        {
                            ring_ctrl.phase_counter++;
                            yanshen_count = ang_numR;
                        }  
                        // 阶段完成则切换状态
                        if(ring_ctrl.phase_counter > 3)
                        {
                            BU_star_flag=0;
                            out_flag=0;
                            Ring_switch();
                            return;
                        }
                        
                    }
                break;

            case Ring_state6:
                    beep.set_level(1);
                    touxian(n_edge,&n_count);
                    blur_points(n_edge,Lout_edge,n_count,3);
                    L_count=n_count*2;
                    resample_points2(Lout_edge,n_count,n_edge,&L_count,3);
                    guai_mum(n_edge,L_count,ang_L,&ang_numL,&ifzhi_L);
                    if(ang_numL>0)
                    L_count=ang_numL;

                    Ring_end();
                    // //左边界（红色）
                    // for (int i = 0; i < Lout_count; i++)
                    // {
                    //     int x = (int)Luse_edge[i][0];
                    //     int y = (int)Luse_edge[i][1];
                    //     // 判断是否有效区域
                    //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                    //     {
                    //         continue;   //无效区域直接不画
                    //     }
                    //     if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT)
                    //     {
                    //         ips200.draw_point(x, y+16*9, RGB565_RED);
                    //     }
                    // }
                    // // 右边界（蓝色）
                    // for (int i = 0; i < Rout_count; i++)
                    // {
                    //     int x = (int)Ruse_edge[i][0];
                    //     int y = (int)Ruse_edge[i][1];
                    //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                    //     {
                    //         continue;   //无效区域直接不画
                    //     }
                    //     if (x >= 0 && x < UVC_WIDTH && y >= 0 && y < UVC_HEIGHT)
                    //     {
                    //         ips200.draw_point(x, y+16*9, RGB565_BLUE);
                    //     }                          
                    // }

                  
                    if(ring_ctrl.kind==left_ring )
                    {          
                        if(lost_r==0)
                        {        
                            ring_ctrl.phase_counter++;
                           
                        }
                        else if(ring_ctrl.phase_counter > 0)
                        {
                            ring_ctrl.phase_counter--;
                         
                        }     
                    }                                                                 
                    else if (ring_ctrl.kind==right_ring)
                    {
                        if(lost_l==0)
                        {        
                            ring_ctrl.phase_counter++;
                          
                        }
                        else if(ring_ctrl.phase_counter > 0)
                        {
                            ring_ctrl.phase_counter--;
                            
                        }    
                    } 
                    
                    if(ring_ctrl.phase_counter>3)
                    {
                        Ring_switch();
                    }
                break;
                 case Ring_state7:
                 beep.set_level(0);
                    touxian(L_edge,&L_count);
                    touxian(R_edge,&R_count);
                    blur_points(L_edge,Lout_edge,L_count,3);
                    blur_points(R_edge,Rout_edge,R_count,3);
                    Lout_count=L_count*2;
                    Rout_count=R_count*2;
                    resample_points2(Lout_edge,L_count,Luse_edge,&Lout_count,3);
                    resample_points2(Rout_edge,R_count,Ruse_edge,&Rout_count,3);
                    guai_mum(Luse_edge,Lout_count,ang_L,&ang_numL,&ifzhi_L);
                    guai_mum(Ruse_edge,Rout_count,ang_R,&ang_numR,&ifzhi_R);
                    if (ang_numL>0)
                    {
                        Lout_count=ang_numL;
                        // int x = (int)Luse_edge[ang_numL][0];
                        // int y = (int)Luse_edge[ang_numL][1];
                        // if (x >= 3 && x < UVC_WIDTH-3 && y >= 3 && y < UVC_HEIGHT-3)
                        // {
                        //     ips200.draw_point(x-1, y+16*9, RGB565_GREEN);
                        //     ips200.draw_point(x, y+16*9, RGB565_GREEN);
                        //     ips200.draw_point(x+1, y+16*9, RGB565_GREEN);
                        //     ips200.draw_point(x, y+16*9-1, RGB565_GREEN);
                        //     ips200.draw_point(x, y+16*9+1, RGB565_GREEN);
                        // }
                    }
                    if (ang_numR>0)
                    {
                        Rout_count=ang_numR;
                        // int a = (int)Ruse_edge[ang_numR][0];
                        // int b = (int)Ruse_edge[ang_numR][1];
                        // if (a >= 3 && a < UVC_WIDTH-3 && b >= 3 && b < UVC_HEIGHT-3)
                        // {
                        //     ips200.draw_point(a-1, b+16*9, RGB565_GREEN);
                        //     ips200.draw_point(a, b+16*9, RGB565_GREEN);
                        //     ips200.draw_point(a+1, b+16*9, RGB565_GREEN);
                        //     ips200.draw_point(a, b+16*9-1, RGB565_GREEN);
                        //     ips200.draw_point(a, b+16*9+1, RGB565_GREEN);
                        // }
                    }        
                       if(ring_ctrl.kind==left_ring )
                    {          
                        if(lost_l)
                        {        
                            ring_ctrl.phase_counter++;
                           
                        }
                        else if(ring_ctrl.phase_counter > 0)
                        {
                            ring_ctrl.phase_counter--;
                         
                        }     
                    }                                                                 
                    else if (ring_ctrl.kind==right_ring)
                    {
                        if(lost_r)
                        {        
                            ring_ctrl.phase_counter++;
                          
                        }
                        else if(ring_ctrl.phase_counter > 0)
                        {
                            ring_ctrl.phase_counter--;
                            
                        }    
                    } 
                    
                    if(ring_ctrl.phase_counter>3)
                    {                       
                        ring_ctrl.phase_counter = 0;
                        ring_ctrl.sum_zangle = 0;
                        ring_ctrl.state=Ring_state1;
                        ring_ctrl.kind = no_ring; 
                        beep.set_level(0);
                    }
                break;
        }
    }
}


// /**
//  * @brief      将输入点数组的一段进行“回滚”复制到输出数组
//  * 
//  * 这个函数通常用于圆环轨迹处理，把 pts_in 中 [1, location2-1] 的点
//  * 倒序复制到 pts_out，从 location1 开始写入，保证不超过 step_Max。
//  *
//  * @param[in]  pts_in     输入点数组，二维数组，每个点包含 [x, y]
//  * @param[out] pts_out    输出点数组，二维数组
//  * @param[in]  location1  输出数组起始位置（索引），开始写入的位置
//  * @param[in]  location2  输入数组回滚结束位置（索引），从这里开始向前复制
//  * @param[in]  step_Max   输出数组最大容量，防止越界
//  *
//  *    返回最后写入输出数组的位置索引（下一个可用位置）
//  */
uint8 Arry_rollback(float pts_in[][2],float pts_out[][2],uint8 location1,uint8 location2,uint8 step_Max)
{
    uint8 j = location1;
    for(int16 i = location2 - 1; i > 0; i--)
    {
        pts_out[j][0] = pts_in[i][0];
        pts_out[j++][1] = pts_in[i][1];
        if(j >= step_Max)break;
    }
    return j;
}

// /**
//  * @brief      圆环状态演化函数
//  * 
//  * 处理圆环状态 Ring_state3 和 Ring_state4 的轨迹演化逻辑。
//  * 根据左右边线状态，生成对边的贝塞尔曲线轨迹，并更新状态机。
//  *
//  * 核心作用：
//  * 1. 判断圆环是否达到切换条件
//  * 2. 根据边线原始点生成对边轨迹
//  * 3. 调整轨迹以保证平滑过渡
//  */
void Ring_evolve(void)
{
    BU_star_flag=0;
    // 先尝试状态切换（如果 phase_counter 达标）
    Ring_switch();
    // ips200.show_int(0,16*18,n_count,3);
    // ips200.show_int(100,16*18,w_count,3);
    // printf("%d,%d\n",n_count,w_count);
    // -----------------------------------------------
    // 情况 1: 对边边线扫描步数超过一半且当前状态是 Ring_state4
    // -----------------------------------------------
    if(w_count > Step_Max / 2 && ring_ctrl.state == Ring_state4) 
    {
        ring_ctrl.phase_counter++;  // 阶段计数 +1

        // 如果累积阶段计数和累积旋转角度超过阈值，则切换状态
        if(ring_ctrl.phase_counter > 3 && fabs(ring_ctrl.sum_zangle) > 45)//车大于45度
        {
            Ring_switch();
            return;
        }
    }
    // -----------------------------------------------
    // 情况 2: 当前边线扫描步数小于 Step_Max / 2
    // -----------------------------------------------
    else if(n_count < Step_Max / 2)
    {
        // Ring_state3 特殊处理：对边点过少，累积 phase_counter
        if(ring_ctrl.state == Ring_state3) 
        {
            if(w_count < MIN_POINTS)
                ring_ctrl.phase_counter++;
            if(ring_ctrl.phase_counter > 3)
            {
                Ring_switch();
                return;
            }
        }
        
        // 生成起点 start[]
        uint8 start[2] = {0};
        start[1] = Start_high; // 高度基准

        if(n_count > 3) {
            // 当前边线有原始点 → 用第一个点加上 width_base(赛道宽度) 作为起点 x
            int16 temp_x = n_edge[0][0] + (ring_ctrl.kind == left_ring ? width_base : -width_base);
            start[0] = temp_x < 3 ? 3 : (temp_x > UVC_WIDTH - 3 ? UVC_WIDTH - 3 : temp_x);
        }
        else if(w_count > Step_Max / 3)
        {
            // 当前边线无点，但对边有足够点 → 用对边第一个点
            start[0] = w_edge[0][0];
            start[1] = w_edge[0][1];
        }
        else
        {
            // 当前边线和对边边线都不足 → 使用中线偏移作为起点
            int16 temp_x = 80 + (ring_ctrl.kind == left_ring ? width_base / 2 : -width_base / 2);
            start[0] = temp_x < 3 ? 3 : (temp_x > UVC_WIDTH - 3 ? UVC_WIDTH - 3 : temp_x);
        }
        // 生成控制点 point[]
        uint8 point[2] = {0};   

        // 上移搜索种子
        seek_up(ring_ctrl.kind);  // 搜索上方轨迹点
        if(BU_star_flag)
        {
            bool bu_couwei=0;
            uint8 turn_point[2]={0};
            uint8 corner_id=0;
            if(ring_ctrl.kind == left_ring)
            {
                touxian(L_edge,&L_count);//内线
                blur_points(L_edge,Lout_edge,L_count,3);
                Lout_count=L_count*2;
                resample_points2(Lout_edge,L_count,Luse_edge,&Lout_count,3);
                guai_mum(Ruse_edge,Rout_count,ang_R,&ang_numR,&ifzhi_R);

                touxian(Bu_R_edge,&Bu_R_count);// 投影变换  //外线
                 // 右边界
                blur_points(Bu_R_edge,Bu_Rout_edge,Bu_R_count,3);
                //**********************************边线重采样
                Bu_Rout_count=Bu_R_count*2;
                resample_points2(Bu_Rout_edge,Bu_R_count,Bu_Ruse_edge,&Bu_Rout_count,3);
                guai_mum(Bu_Ruse_edge,Bu_Rout_count,Bu_ang_R,&Bu_ang_numR,&bu_couwei); // 寻找转折点
    
                // 转折点坐标
                turn_point[0] = Bu_Ruse_edge[Bu_ang_numR][0];
                turn_point[1] = Bu_Ruse_edge[Bu_ang_numR][1];
                corner_id=Bu_ang_numR;
                if(Bu_ang_numR<1)
                {
                    turn_point[0]=Bu_Ruse_edge[Bu_Rout_count][0];
                    turn_point[0]=Bu_Ruse_edge[Bu_Rout_count][1];
                    corner_id=Bu_Rout_count;
                }                
            }
            else if(ring_ctrl.kind == right_ring)
            {           
                touxian(R_edge,&R_count);
                blur_points(R_edge,Rout_edge,R_count,3);
                Rout_count=R_count*2;
                resample_points2(Rout_edge,R_count,Ruse_edge,&Rout_count,3);
                guai_mum(Ruse_edge,Rout_count,ang_R,&ang_numR,&ifzhi_R);

                touxian(Bu_L_edge,&Bu_L_count);// 投影变换
                //**********************************边线滤波
                blur_points(Bu_L_edge,Bu_Lout_edge,Bu_L_count,3);
                //**********************************边线重采样
                Bu_Lout_count=Bu_L_count*2;
                resample_points2(Bu_Lout_edge,Bu_L_count,Bu_Luse_edge,&Bu_Lout_count,3);
                guai_mum(Bu_Luse_edge,Bu_Lout_count,Bu_ang_L,&Bu_ang_numL,&bu_couwei); // 寻找转折点
                // 转折点坐标
                turn_point[0]=Bu_Luse_edge[Bu_ang_numL][0];
                turn_point[1]=Bu_Luse_edge[Bu_ang_numL][1];
                corner_id=Bu_ang_numL;
                if(Bu_ang_numL<1)
                {
                    turn_point[0]=Bu_Luse_edge[Bu_Lout_count][0];
                    turn_point[0]=Bu_Luse_edge[Bu_Lout_count][1];
                    corner_id=Bu_Lout_count;
                }
                
            }
            // -------------------------
            // 生成对边贝塞尔曲线
            // -------------------------
            int a=0; int b=0;
            a=Xrot_point(start[0],start[1]);
            b=Yrot_point(start[0],start[1]);
            start[0]=a;
            start[1]=b;

            point[0] = (start[0] + turn_point[0]) / 2;
            point[1] = turn_point[1] * 1.3 < Start_high ? turn_point[1] * 1.3 : Start_high;

            yanshen_count = add_three_point_bezier(
                start, point, turn_point,
                yanshen,
                0, 1
            );

            // -------------------------
            // 倒序回滚，确保尾部连续
            // -------------------------
            if(ring_ctrl.kind == left_ring)
            {
                yanshen_count = Arry_rollback(
                    Bu_Ruse_edge,
                    yanshen,
                    yanshen_count,
                    corner_id,
                    UVC_WIDTH
                );
                blur_points(yanshen,Bu_Ruse_edge,yanshen_count,3);
                Rout_count=yanshen_count*2;
                resample_points(Bu_Ruse_edge,yanshen_count,Ruse_edge,&Rout_count,3);//用于边线合成
                // // 右边界（蓝色）
                // for (int i = 0; i < yanshen_count; i++)
                // {
                //     int x = (int)yanshen[i][0];
                //     int y = (int)yanshen[i][1];
                //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                //     {
                //         continue;   //无效区域直接不画
                //     }
                //     if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
                //     {
                //         ips200.draw_point(x, y+16*9, RGB565_BLUE);
                //         ips200.draw_point(x+1, y+16*9, RGB565_BLUE);
                //     } 
                // }      
            }
            else if(ring_ctrl.kind == right_ring)
            {
                yanshen_count = Arry_rollback(
                    Bu_Luse_edge,
                    yanshen,
                    yanshen_count,
                    corner_id,
                    UVC_WIDTH
                );
                blur_points(yanshen,Bu_Luse_edge,yanshen_count,3);
                Lout_count=yanshen_count*2;
                resample_points(Bu_Luse_edge,yanshen_count,Luse_edge,&Lout_count,3);//用于边线合成
                // for (int i = 0; i < yanshen_count; i++)
                // {
                //     int x = (int)yanshen[i][0];
                //     int y = (int)yanshen[i][1];
                //     // 判断是否有效区域
                //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                //     {
                //         continue;   //无效区域直接不画
                //     }
                //     if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
                //     {
                //         ips200.draw_point(x, y+16*9, RGB565_RED);
                //         ips200.draw_point(x+1, y+16*9, RGB565_RED);
                //     }
                // }
            }
        }
    }
        // -----------------------------------------------
        // 情况 3: 当前边线步数 >= Step_Max / 2
        // -----------------------------------------------
    else
    {
        if(ring_ctrl.state == Ring_state3) 
        {
            if(w_count < MIN_POINTS)
                ring_ctrl.phase_counter++;
            if(ring_ctrl.phase_counter > 3)
            {
                Ring_switch();
                return;
            }
        }
        // 重置对边当前扫描步数
        w_count = 0;
    }
}


uint8 IDID = 0; 
int end_point[2]={0};
void Ring_Out(void)
{
    BU_flag=0;
    BU_star_flag=0;
    Ring_switch();   // 检查phase_counter是否达到阈值，如果达到则切换到下一个状态
    // 判断拐角点是否在边界点前2/3范围内，并且不是最后几个点
    if( ang_numR < R_count - 3 && ang_numR>0)//ang_numR < R_count * 2 / 3 &&
    {
        // 在拐角点附近构造一个新的搜索起点（稍微向内偏移）
        // 从该点向上重新搜线
        int BU_w=w_edge[ang_numR][0]+(ring_ctrl.kind == left_ring ? -5 : 5);
        Bu_R_edge[0][0]=w_edge[ang_numR][0]+(ring_ctrl.kind == left_ring ? -5 : 5); 
        for(int16 i = (int16)w_edge[ang_numR][1];i>30;i--)
        {
            if (image_use[i][BU_w] >= yuzhi && image_use[i-1][BU_w]<yuzhi&&image_use[i-2][BU_w]<yuzhi) // 白黑黑跳变
            {
                Bu_R_edge[0][1] = i;   // 记录右边界行号
                BU_flag=1;
                end_point[0]=w_edge[ang_numR][0];
                end_point[1]=w_edge[ang_numR][1];//更新最终角点
                break;       // 找到后停止扫描
            }
        }
        if(BU_flag)
        {
            if(ring_ctrl.kind == left_ring)
            get_BLY(Bu_L_edge,&Bu_L_count,Bu_R_edge,&Bu_R_count,30,0,1);
            else if(ring_ctrl.kind == right_ring)
            get_BLY(Bu_R_edge,&Bu_R_count,Bu_L_edge,&Bu_L_count,30,1,0);
            if(Bu_R_count>3)
            {
                BU_star_flag=1;
                out_flag=1;              
            }
            //   for (int i = 0; i < Bu_R_count; i++)
            // {
            //     int x = (int)Bu_R_edge[i][0];
            //     int y = (int)Bu_R_edge[i][1];
            //     if (x > 0 && x < UVC_WIDTH-1 && y > 0 && y < UVC_HEIGHT)
            //     {
            //         ips200.draw_point(x, y, RGB565_GREEN);
            //         ips200.draw_point(x+1, y, RGB565_GREEN);
            //     }
            // }
            if(BU_star_flag)
            {
                touxian(Bu_R_edge,&Bu_R_count);           
                uint8 index = 0;
                // 计算corner点与起始点的高度差
                uint8 h1 = abs(w_edge[ang_numR][1] - w_edge[0][1]);
                // 计算corner点与起始点的宽度差
                uint8 w1 = abs(w_edge[ang_numR][0] - w_edge[0][0]);
                // 计算新搜边界的高度差
                uint8 h2 = abs(Bu_R_edge[0][1] - w_edge[ang_numR][1]);
                // 根据相似三角形估计新的横向距离
                uint8 w2 = w1 * h2 / h1;
                // 根据左右环方向计算预测x坐标
                int16 temp_mx = w_edge[ang_numR][0] + (ring_ctrl.kind == left_ring ? -w2 : w2);
                uint8 MX;
                // 限制预测点在图像范围内
                if(temp_mx < 0) 
                    MX = 0;
                else if(temp_mx > UVC_WIDTH - 1) 
                    MX = UVC_WIDTH - 1;
                else 
                    MX = (uint8)temp_mx;

                // 在新搜到的边界中寻找接近该预测x的位置
                for(uint8 i = 0; i < Bu_R_count; i++)
                {
                    uint8 x = Bu_R_edge[i][0];

                    if((ring_ctrl.kind == left_ring && x <= MX) || (ring_ctrl.kind == right_ring && x >= MX))
                    {
                        index = i;
                        break;
                    }
                }

                // 取新边界中间点作为贝塞尔曲线终点
                uint8 turn_point[2] = {
                    Bu_R_edge[Bu_R_count / 2][0],
                    Bu_R_edge[Bu_R_count / 2][1]
                };

                // 将point改为新边界上的连接点
                uint8 point[2];
                point[0] = Bu_R_edge[index][0];
                point[1] = Bu_R_edge[index][1];

                uint8 start[2];
                start[0]=w_edge[ang_numR][0];
                start[1]=w_edge[ang_numR][1];

                // 记录环形结束高度（用于后续阶段判断）
                ring_ctrl.last_high = (w_edge[ang_numR][1] + point[1]) / 2;
                // 使用三点贝塞尔曲线补全对侧边界
                yanshen_count = add_three_point_bezier(   
                    start,  // 起点
                    point,                                              // 中间控制点
                    turn_point,                                         // 终点
                    yanshen,
                    0,
                    1
                ) ;

                // 将新搜到的边界剩余部分拼接到贝塞尔曲线后
                yanshen_count = Arry_roll(
                    yanshen,
                    w_edge,
                    ang_numR,
                    1,
                    yanshen_count,
                    150
                );
                // for (int i = 0; i < yanshen_count; i++)
                // {
                //     int x = (int)w_edge[i][0];
                //     int y = (int)w_edge[i][1];
                //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
                //     {
                //         continue;   //无效区域直接不画
                //     }
                //     if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
                //     {
                //         ips200.draw_point(x, y+16*9, RGB565_YELLOW);
                //         ips200.draw_point(x+1, y+16*9, RGB565_YELLOW);
                //     } 
                // }
                if(ring_ctrl.kind == left_ring)//用于边线合成
                {

                    blur_points(w_edge,Rout_edge,yanshen_count,3);
                    resample_points(Rout_edge,yanshen_count,Ruse_edge,&Rout_count,3);

                    memcpy(Luse_edge, n_edge, sizeof(n_edge));
                    Lout_count=L_count;
                }
                else if(ring_ctrl.kind == right_ring)
                {
                    Rout_count=L_count;
                    memcpy(Ruse_edge, n_edge, sizeof(n_edge));

                    blur_points(w_edge,Rout_edge,yanshen_count,3);
                    resample_points(Rout_edge,yanshen_count,Luse_edge,&Lout_count,3);
                }
            }
        }
    }
    // else
    // {
    //     // 如果边界点太少，直接截断
    //     yanshen_count = ang_numR;
    // }
    // // 更新当前使用的边界点（用于中线合成）
    // (ring_ctx.is_left ? RightPts_Deal_Now : LeftPts_Deal_Now)(); 
}

void Ring_end(void)
{
    Ring_switch();   // 检查phase_counter是否达到阈值，如果达到则切换到下一个状态
    int x = end_point[0];
    int y = end_point[1];
    // if (x >= 3 && x < UVC_WIDTH-3 && y >= 3 && y < UVC_HEIGHT-3)
    // {
    //     ips200.draw_point(x-1, y+16*9, RGB565_GREEN);
    //     ips200.draw_point(x, y+16*9, RGB565_GREEN);
    //     ips200.draw_point(x+1, y+16*9, RGB565_GREEN);
    //     ips200.draw_point(x, y+16*9-1, RGB565_GREEN);
    //     ips200.draw_point(x, y+16*9+1, RGB565_GREEN);
    // }
    // 在最终角点附近构造一个新的搜索起点（稍微向内偏移）
    // 从该点向上重新搜线
    int BU_w=end_point[0]+(ring_ctrl.kind == left_ring ? -5 : 5);
    Bu_R_edge[0][0]=end_point[0]+(ring_ctrl.kind == left_ring ? -5 : 5); 
    for(int16 i = (int16)end_point[1];i>30;i--)
    {
        if (image_use[i][BU_w] >= yuzhi && image_use[i-1][BU_w]<yuzhi&&image_use[i-2][BU_w]<yuzhi) // 白黑黑跳变
        {
            Bu_R_edge[0][1] = i;   // 记录右边界行号
            BU_flag=1;
            break;       // 找到后停止扫描
        }
    }
    if(BU_flag)
    {
        if(ring_ctrl.kind == left_ring)
        get_BLY(Bu_L_edge,&Bu_L_count,Bu_R_edge,&Bu_R_count,30,0,1);
        else if(ring_ctrl.kind == right_ring)
        get_BLY(Bu_R_edge,&Bu_R_count,Bu_L_edge,&Bu_L_count,30,1,0);
        if(Bu_R_count>3)
        {
            BU_star_flag=1;
            out_flag=1;              
        }
        //     for (int i = 0; i < Bu_R_count; i++)
        // {
        //     int x = (int)Bu_R_edge[i][0];
        //     int y = (int)Bu_R_edge[i][1];
        //     if (x > 0 && x < UVC_WIDTH-1 && y > 0 && y < UVC_HEIGHT)
        //     {
        //         ips200.draw_point(x, y, RGB565_GREEN);
        //         ips200.draw_point(x+1, y, RGB565_GREEN);
        //     }
        // }
        if(BU_star_flag)
        {
            touxian(Bu_R_edge,&Bu_R_count);         

            // 取新边界中间点作为贝塞尔曲线终点
            uint8 turn_point[2] = {
                Bu_R_edge[Bu_R_count / 2][0],
                Bu_R_edge[Bu_R_count / 2][1]
            };
            uint8 start[2];
            start[0]=end_point[0];
            start[1]=end_point[1];

            // 将point改为新边界上的连接点
            uint8 point[2];
            point[0] = (turn_point[0]+start[0])/2;
            point[1] = (turn_point[1]+start[1])/2;

            // 记录环形结束高度（用于后续阶段判断）
            ring_ctrl.last_high = (end_point[1] + point[1]) / 2;
            // 使用三点贝塞尔曲线补全对侧边界
            yanshen_count = add_three_point_bezier(   
                start,  // 起点
                point,      // 中间控制点
                turn_point,   // 终点
                yanshen,
                0,
                1
            ) ;

            if(ring_ctrl.kind == left_ring)//用于边线合成
            {
                blur_points(yanshen,Ruse_edge,yanshen_count,3);
                 
                // resample_points(Rout_edge,yanshen_count,Ruse_edge,&Rout_count,3);//重采样完就没了，试试不重采
                Rout_count=yanshen_count;
            
                  
                memcpy(Luse_edge, n_edge, sizeof(n_edge));
                Lout_count=L_count;
            }
            else if(ring_ctrl.kind == right_ring)
            {
                Rout_count=L_count;
                memcpy(Ruse_edge, n_edge, sizeof(n_edge));

                blur_points(yanshen,Luse_edge,yanshen_count,3);
                Lout_count=yanshen_count;
                // resample_points(Lout_edge,yanshen_count,Luse_edge,&Lout_count,3); 
            }
        }
    }

}
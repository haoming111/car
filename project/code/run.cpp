#include "run.hpp"
#include "math.h"
#include "camera_send.hpp"
#include "image.hpp"
#include "encoder.hpp"
#include "ring.hpp"
#include "key.hpp"
#include "zebra.hpp"
#include "red.hpp"
#include "rot.hpp"

extern zf_device_ips200 ips200;
extern int red_state;

int qianzhan=10;  //18         //设定预锚点距离
int qian=12;
uint8 yuzhi=0;

float L_edge[100][2];     //左边界结构体                                //我的
float Lout_edge[100][2];     //左边界结构体
float Luse_edge[100][2];
float R_edge[100][2];    //右边界结构体
float Rout_edge[100][2];     //左边界结构体
float Ruse_edge[100][2];
float ang_L[100],ang_R[100];                                           //我的

int L_count=0, R_count = 0,Lout_count=0,Rout_count=0; //左右边点的个数 

int ang_numL=0,ang_numR=0;
bool ifzhi_L=0,ifzhi_R=0;
bool lost_l=0;
bool lost_r=0;
int souxian=30;//30
extern uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
extern uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];
void search_bianxian()
{
    get_highest();
    get_BLY(L_edge,&L_count,R_edge,&R_count,souxian,0,0);

    lost_l=0;
    lost_r=0;
    if(L_count< 4)
    {
       lost_l=1;
    }
    if(R_count < 4)
    {
        lost_r=1;
    }
    if(car_star==0)
    {
        //左边界（红色）
        for (int i = 0; i < L_count; i++)
        {
            int x = (int)L_edge[i][0];
            int y = (int)L_edge[i][1];
            // 判断是否有效区域
            if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
            {
                continue;   //无效区域直接不画
            }
            if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
            {
                ips200.draw_point(x, y, RGB565_RED);
                ips200.draw_point(x+1, y, RGB565_RED);
            }
        }
        // 右边界（蓝色）
        for (int i = 0; i < R_count; i++)
        {
            int x = (int)R_edge[i][0];
            int y = (int)R_edge[i][1];
                if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
            {
                continue;   //无效区域直接不画
            }
            if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
            {
                ips200.draw_point(x, y, RGB565_BLUE);
                ips200.draw_point(x+1, y, RGB565_BLUE);
            }
        }
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
    resample_points(Lout_edge,L_count,Luse_edge,&Lout_count,3);
    resample_points(Rout_edge,R_count,Ruse_edge,&Rout_count,3);
                       
    //**********************************找拐点
    guai_mum(Luse_edge,Lout_count,ang_L,&ang_numL,&ifzhi_L);
    guai_mum(Ruse_edge,Rout_count,ang_R,&ang_numR,&ifzhi_R);
                    
    if (ang_numL>0)
    {
        Lout_count=ang_numL;

    //     int x = (int)Luse_edge[ang_numL][0];
    //     int y = (int)Luse_edge[ang_numL][1];
    //     if (x >= 3 && x < UVC_WIDTH-3 && y >= 3 && y < UVC_HEIGHT-3)
    //     {
    //         ips200.draw_point(x-1, y+16*9, RGB565_GREEN);
    //         ips200.draw_point(x, y+16*9, RGB565_GREEN);
    //         ips200.draw_point(x+1, y+16*9, RGB565_GREEN);
    //         ips200.draw_point(x, y+16*9-1, RGB565_GREEN);
    //         ips200.draw_point(x, y+16*9+1, RGB565_GREEN);
    //     }
    }
    if (ang_numR>0)
    {
        Rout_count=ang_numR;

    //     int a = (int)Ruse_edge[ang_numR][0];
    //     int b = (int)Ruse_edge[ang_numR][1];
    //     if (a >= 3 && a < UVC_WIDTH-3 && b >= 3 && b < UVC_HEIGHT-3)
    //     {
    //         ips200.draw_point(a-1, b+16*9, RGB565_GREEN);
    //         ips200.draw_point(a, b+16*9, RGB565_GREEN);
    //         ips200.draw_point(a+1, b+16*9, RGB565_GREEN);
    //         ips200.draw_point(a, b+16*9-1, RGB565_GREEN);
    //         ips200.draw_point(a, b+16*9+1, RGB565_GREEN);
    //     }
    }
}


float Lin_MID[50][2];     //左中线结构体
float Lout_MID[50][2];
float Rin_MID[50][2];     //左中线结构体
float Rout_MID[50][2];
int Lmidnum,Lmidnum2;
int Rmidnum,Rmidnum2;

int xunxian_dir=0;
void zhongxianhecheng()
{
    //**********************************边线采集中线
    if (Lout_count<50)
    Lmidnum=Lout_count;
    else 
    Lmidnum=49;
    if (Rout_count<50)
    Rmidnum=Rout_count;
    else 
    Rmidnum=49;

    get_mid_L(Luse_edge,Lmidnum,Lin_MID,&Lmidnum,2);
    get_mid_R(Ruse_edge,Rmidnum,Rin_MID,&Rmidnum,2);
    //**********************************中线滤波
    blur_points(Lin_MID,Lout_edge,Lmidnum,3);
    blur_points(Rin_MID,Rout_edge,Rmidnum,3);
    //**********************************重采样中线
    if (Lmidnum*2<49)Lmidnum2=Lmidnum*2;
    else Lmidnum2=50;
    if (Rmidnum*2<49)Rmidnum2=Rmidnum*2;
    else Rmidnum2=50;
    resample_points(Lout_edge,Lmidnum,Lout_MID,&Lmidnum2,4);//4
    resample_points(Rout_edge,Rmidnum,Rout_MID,&Rmidnum2,4);    

    if (Lmidnum2>Rmidnum2)
    {
        xunxian_dir=-1;
    }
    else
    {
        xunxian_dir=1;
    }
}


int shizi=0,len_shizi=0;

int luzhang=0;
int luzhangjuli=0;
int dir_luzhang=0;

int yanshi=0;
int juli=0;

int banmacishu=0;
int qipao=0;
int qipaojuli=0;

int podao=0;
int pochang=0;
int poyanshi=0;

int jian=0,jianms=0;
// void xunxain_celue()
// {

//     if (Lmidnum>Rmidnum)
//     {
//         xunxian_dir=-1;
//     }
//     else
//     {
//         xunxian_dir=1;
//     }

//     if (yuanhuan==1)
//     {
//         xunxian_dir=-dir_yuanhuan;
//     }

//     if (yuanhuan==2)
//     {
//         /**/
//         if (dir_yuanhuan==1)
//         {
//             if (Rout_count>3)
//             {
//                 xunxian_dir=dir_yuanhuan;
//             }
//             // else if (ang_numL==0&&suo==0)
//             // {
//             //     xunxian_dir=-dir_yuanhuan;
//             // }
//             else if(ifzhi_L==0)
//             {
//                 // suo=1;
//                 xunxian_dir=-dir_yuanhuan;
//             }
//         }


//         if (dir_yuanhuan==-1)
//         {
//             if (Lout_count>5)
//             {
//                 xunxian_dir=dir_yuanhuan;
//             }
//             // else if (ang_numR==0&&suo==0)
//             // {
//             //     xunxian_dir=-dir_yuanhuan;
//             // }
//             else if(ifzhi_L==0)
//             {
//                 // suo=1;
//                 xunxian_dir=-dir_yuanhuan;
//             }
//         }

//     }

//     if (yuanhuan==3)
//     {
//         xunxian_dir=-dir_yuanhuan;
//     }

//     if (luzhang==1)
//     {
//         xunxian_dir=-dir_luzhang;
//         if (xunxian_dir==1)
//         {
//             Rout_MID[qianzhan][0]+=20;
//         }
//         else if (xunxian_dir==-1)
//         {
//             Lout_MID[qianzhan][0]-=20;
//         }
//     }


//     //tiaoshi
//     // if(juli>65530)
//     // {
//     //     juli=0;
//     // }
//     //      if (ifzhi_L==1&&ang_numR>0&&ang_numR<5)
//     //     {
//     //         wodeyuanhuan=1;
//     //     }
//     //     if(wodeyuanhuan==1)
//     //     juli+=Measure_Speedl-Measure_Speedr;
//     ips200.show_int(300,16*19,yuanhuan, 1);
    
// }

void xunxain_celue()
{
    if (ring_ctrl.state==1||(ring_ctrl.state>2&&ring_ctrl.state<5)||ring_ctrl.state==6)//1 3  4  6 wai
    {
          if(ring_ctrl.kind == left_ring)
        {
            if(Rout_count>5)
            xunxian_dir=1;
            else
            xunxian_dir=-1;
          
        }
        else if(ring_ctrl.kind == right_ring)
        {
            if(Lout_count>5)
            xunxian_dir=-1;
            else
            xunxian_dir=1;
           
        }
    }
    else if ( ring_ctrl.state==2||ring_ctrl.state==5)// 2   5
    {
          if(ring_ctrl.kind == right_ring)
        {
            if(Rout_count>5)
            xunxian_dir=1;
            else
            xunxian_dir=-1;

        }
        else if(ring_ctrl.kind == left_ring)
        {
            if(Lout_count>5)
            xunxian_dir=-1;
            else
            xunxian_dir=1;
        }
    }

    if (luzhang==1)
    {
        xunxian_dir=-dir_luzhang;
        if (xunxian_dir==1)
        {
            Rout_MID[qianzhan][0]+=20;
        }
        else if (xunxian_dir==-1)
        {
            Lout_MID[qianzhan][0]-=20;
        }
    }
}

float get_angle()
{
    float angle=0;
    if (xunxian_dir==-1)             //dir=-1代表巡左边线
    {
        if (Lmidnum2>qianzhan)       //如果左中线长度长于预锚点距离，使用预锚点计算误差
        { 
            float dx=Lout_MID[qianzhan][0]-UVC_WIDTH/2,dy=UVC_HEIGHT+20-Lout_MID[qianzhan][1];
            angle=1*atan2f(dx, dy) * 180 / PI;
            // printf("11  %f,%f\n",dx,dy);
        }
        else                                           //如果左中线长度短于预锚点距离，使用最远中线点点计算误差
        {
            float dx=Lout_MID[Lmidnum2][0]-UVC_WIDTH/2,dy=UVC_HEIGHT+20-Lout_MID[Lmidnum2][1];
            angle=1*atan2f(dx, dy) * 180 / PI;
            // printf("12  %f,%f\n",dx,dy);
        }
    }
    else if (xunxian_dir==1)      //dir=1代表巡右边线
    {
        if (Rmidnum2>qianzhan)    //如果右中线长度长于预锚点距离，使用预锚点计算误差
        {
            float dx=Rout_MID[qianzhan][0]-UVC_WIDTH/2,dy=UVC_HEIGHT+20-Rout_MID[qianzhan][1];
            angle=1*atan2f(dx, dy) * 180 / PI;
            // printf("21  %f,%f\n",dx,dy);
        }
        else                                         //如果右中线长度短于预锚点距离，使用最远中线点点计算误差
        {
            float dx=Rout_MID[Rmidnum2][0]-UVC_WIDTH/2,dy=UVC_HEIGHT+20-Rout_MID[Rmidnum2][1];
            angle=1*atan2f(dx, dy) * 180 / PI;
            // printf("22  %f,%f Lmidnum2=%d Rmidnum2=%d\n",dx,dy,Lmidnum2,Rmidnum2);
        }
    }

    return angle;
}


int left_start[2]={0}; 
int right_start[2]={0}; 
bool l_flag=0;
bool r_flag=0;
void if_shizi(void)
{
    if (podao==0&&luzhang==0&& ring_ctrl.kind == no_ring &&shizi==0 && zebra_state==0&& red_state==0)
    {
        if(ang_numR>0&&ang_numR<5&&ang_numL>0&&ang_numL<5)//得到的角点是透视后的
        {
            shizi=3;
            left_start[0]=Luse_edge[ang_numL][0]-10;
            left_start[1]=Luse_edge[ang_numL][1]-22;
            if(left_start[0]<0)
            {
                left_start[0]=1;
            }
            right_start[0]=Ruse_edge[ang_numR][0]+10;
            right_start[1]=Ruse_edge[ang_numR][1]-22;
              if(right_start[0]>UVC_WIDTH)
            {
                right_start[0]=UVC_WIDTH-1;
            }
        }
        else if (ang_numR>0&&ang_numR<5&&Lout_count<8)
        {
            shizi=2;
            right_start[0]=Ruse_edge[ang_numR][0]+20;
            right_start[1]=Ruse_edge[ang_numR][1]-25;
            left_start[0]=right_start[0]-38-40;
            if(left_start[0]<0)
            {
                left_start[0]=1;
            }
            left_start[1]=right_start[1];
        }
        else if(ang_numL>0&&ang_numL<5&&Rout_count<8)
        {
            shizi=1;
            left_start[0]=Luse_edge[ang_numL][0]-20;
            left_start[1]=Luse_edge[ang_numL][1]-25;
            right_start[0]=left_start[0]+38+40;
            if(right_start[0]>UVC_WIDTH)
            {
                right_start[0]=UVC_WIDTH-1;
            }
            right_start[1]=left_start[1];  
        }
    }

    if (shizi!=0)
    {
        get_highest();
        get_BLY(Bu_L_edge,&Bu_L_count,Bu_R_edge,&Bu_R_count,30,0,0);
        lost_l=0;
        lost_r=0;
        if(Bu_L_count < 4)
        {
            lost_l=1;
        }
        if(Bu_R_count < 4)
        {
            lost_r=1;
        }

        if(shizi<4)
        {
            beep.set_level(1);
            if(lost_l&&lost_r)
            {
                shizi=4;
            }
        }

        len_shizi+=Measure_Speedl-Measure_Speedr;
        // ips200.draw_line(       //搜线起始行
        //             (uint16)left_start[0],
        //             (uint16)left_start[1],
        //             (uint16)right_start[0],
        //             (uint16)right_start[1],
        //             RGB565_RED
        //         );  

        int BU_L=left_start[0];
        L_edge[0][0]=left_start[0]; 
        for(int i = left_start[1];i>30;i--)
        {
            if (image_use[i][BU_L] >= yuzhi&&image_use[i-1][BU_L]<yuzhi&&image_use[i-2][BU_L]<yuzhi) // 白黑黑跳变
            {
                L_edge[0][1] = i;   // 记录右边界行号
                // l_flag=1;
                break;       // 找到后停止扫描
            }
        }    
        int BU_R=right_start[0];
        R_edge[0][0]=right_start[0]; 
        for(int i = right_start[1];i>30;i--)
        {
            if (image_use[i][BU_R] >= yuzhi&&image_use[i-1][BU_R]<yuzhi&&image_use[i-2][BU_R]<yuzhi) // 白黑黑跳变
            {
                R_edge[0][1] = i;   // 记录右边界行号
                // r_flag=1;
                break;       // 找到后停止扫描
            }
        }   
        // if(l_flag&&r_flag)  
        // {           
            get_BLY(L_edge,&L_count,R_edge,&R_count,30,1,1);
            
            // for (int i = 0; i < L_count; i++)//左边界（红色）
            // {
            //     int x = (int)L_edge[i][0];
            //     int y = (int)L_edge[i][1];
            //     // 判断是否有效区域
            //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
            //     {
            //         continue;   //无效区域直接不画
            //     }
            //     if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
            //     {
            //         ips200.draw_point(x, y, RGB565_RED);
            //         ips200.draw_point(x+1, y, RGB565_RED);
            //     }
            // }

            // // 右边界（蓝色）
            // for (int i = 0; i < R_count; i++)
            // {
            //     int x = (int)R_edge[i][0];
            //     int y = (int)R_edge[i][1];
            //     if(x < 0 ||  x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT)
            //     {
            //         continue;   //无效区域直接不画
            //     }
            //     if (x >= 0 && x < UVC_WIDTH+1 && y >= 0 && y < UVC_HEIGHT)
            //     {
            //         ips200.draw_point(x, y, RGB565_BLUE);
            //         ips200.draw_point(x+1, y, RGB565_BLUE);
            //     }
            // }

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

            guai_mum(Luse_edge,Lout_count,ang_L,&ang_numL,&ifzhi_L);
            guai_mum(Ruse_edge,Rout_count,ang_R,&ang_numR,&ifzhi_R);

            // 如果ang_numL已经是0，无需移动
            if (ang_numL > 0)
            {
                // 计算新长度
                int new_len_l = Lout_count - ang_numL;
                // 将ang_numL及之后的元素前移
                for (int i = 0; i < new_len_l; i++) 
                {
                    Luse_edge[i][0] = Luse_edge[ang_numL + i][0];
                    Luse_edge[i][1] = Luse_edge[ang_numL + i][1];
                }
                Lout_count=new_len_l;
            }
            if (ang_numR > 0)
            {
                int new_len_r = Rout_count - ang_numR;
                for (int i = 0; i < new_len_r; i++) 
                {
                    Ruse_edge[i][0] = Ruse_edge[ang_numR + i][0];
                    Ruse_edge[i][1] = Ruse_edge[ang_numR + i][1];
                }
                Rout_count=new_len_r;
            }
           
        }
    // } 
    if(shizi>3)
    {
        beep.set_level(0);
        left_start[0]=80-45;
        left_start[1]=89;
        right_start[0]=80+45;
        right_start[1]=89;
        if(lost_l==0&&lost_r==0)
        {
            shizi=0;
            len_shizi=0;
        }
    }

    if (len_shizi>4000)//14000
    {
        beep.set_level(0);
        len_shizi=0;
        shizi=0;
    }
    
    // ips200.show_int(50,16*18,shizi, 1);
    // ips200.show_int(0,16*19,l_flag, 1);
    // ips200.show_int(300,16*19,r_flag, 1);
    // ips200.show_int(200,16*18,len_shizi,5);
}

float Bu_L_edge[100][2];     //补线左边界结构体                        
float Bu_Lout_edge[100][2];     //补线边界结构体
float Bu_Luse_edge[100][2];
float Bu_R_edge[100][2];    //右边界结构体
float Bu_Rout_edge[100][2];     //左边界结构体
float Bu_Ruse_edge[100][2];
int Bu_L_count,Bu_R_count,Bu_Lout_count,Bu_Rout_count;
float Bu_ang_L[100],Bu_ang_R[100];   
int Bu_ang_numL=0,Bu_ang_numR=0;
bool BU_flag=0;
bool BU_star_flag=0;
void seek_up(uint8 kind )
{
    if(kind==1)
    {
        int BU_R=L_edge[L_count-2][0]+7;
        Bu_R_edge[0][0]=L_edge[L_count-2][0]+7; 
        for(int16 i = (int16)L_edge[L_count-2][1];i>30;i--)
        {
            if (image_use[i][BU_R] >= yuzhi&&image_use[i-1][BU_R]<yuzhi&&image_use[i-2][BU_R]<yuzhi) // 白黑黑跳变
            {
                Bu_R_edge[0][1] = i;   // 记录右边界行号
                BU_flag=1;
                break;       // 找到后停止扫描
            }
        }
        if(BU_flag)
        {
            get_BLY(Bu_R_edge,&Bu_R_count,Bu_L_edge,&Bu_L_count,30,1,0);
            if(Bu_R_count>5)
            {
                BU_star_flag=1;
            }
            BU_flag=0;
        }
        //  for (int i = 0; i < Bu_R_count; i++)
        // {
        //     int x = (int)Bu_R_edge[i][0];
        //     int y = (int)Bu_R_edge[i][1];
        //     if (x > 0 && x < UVC_WIDTH-1 && y > 0 && y < UVC_HEIGHT)
        //     {
        //         ips200.draw_point(x, y, RGB565_GREEN);
        //         ips200.draw_point(x-1, y, RGB565_GREEN);
        //     }
        // }
    }
    else if (kind==2)
    {
        int BU_L=R_edge[R_count-2][0]-10;//x轴坐标，即第几列
        Bu_L_edge[0][0]=R_edge[R_count-2][0]-10;
        for(int16 i = (int16)R_edge[R_count-2][1];i>30;i--)
        {
            if (image_use[i][BU_L] >= yuzhi&&image_use[i-1][BU_L]<yuzhi&&image_use[i-2][BU_L]<yuzhi) // 白黑黑跳变
            {
                Bu_L_edge[0][1] = i;       
                BU_flag=1;       
                break;     
            }
        }

         if(BU_flag)
        {
            get_BLY(Bu_R_edge,&Bu_R_count,Bu_L_edge,&Bu_L_count,30,0,1);
            if(Bu_L_count>5)
            {
                BU_star_flag=1;
            }
            BU_flag=0;
        }

        // for (int i = 0; i < Bu_L_count; i++)
        // {
        //     int x = (int)Bu_L_edge[i][0];
        //     int y = (int)Bu_L_edge[i][1];
        //     if (x > 0 && x < UVC_WIDTH-1 && y > 0 && y < UVC_HEIGHT)
        //     {
        //         ips200.draw_point(x, y, RGB565_GREEN);
        //         ips200.draw_point(x+1, y, RGB565_GREEN);
        //     }
        // }
    }
   
}

extern int lwline; 

void if_luzhang(void)
{
    if (red_state==0&&podao==0&&qipao==0&&luzhang==0&&ring_ctrl.kind == no_ring&&shizi==0&&(ifzhi_L==1||ifzhi_R==1))
    {
        float Lx=0,Rx=160,lukuan=0;
        float h=inv_rot_y(80,60);//中心点
        h=119-h;
        for (int16 i=(int16)lwline;i>0;i--)
        {

            if ((cbh(i,UVC_HEIGHT-h-1,6)==1 && cbh(i+1,UVC_HEIGHT-h-1,6)==1&&cbh(i+2,UVC_HEIGHT-h-1,6)==1&&cbh(i+3,UVC_HEIGHT-h-1,6)==1&&cbh(i+4,UVC_HEIGHT-h-1,6)==0)||i<=8)
            {
                Lx=i+4;
                break;
            }
        }
        for (int16 i=(int16)lwline;i<UVC_WIDTH;i++)
        {
            if ((cbh(i,UVC_HEIGHT-h-1,2)==1 && cbh(i-1,UVC_HEIGHT-h-1,2)==1&&cbh(i-2,UVC_HEIGHT-h-1,2)==1&&cbh(i-3,UVC_HEIGHT-h-1,2)==1&&cbh(i-4,UVC_HEIGHT-h-1,2)==0)||i>=152)
            {
                Rx=i-4;
                break;
            }
        }
        lukuan=Xrot_point(Rx,UVC_HEIGHT-h-1)-Xrot_point(Lx,UVC_HEIGHT-h-1);
        if (lukuan<=37)
        {
            luzhang=1;
            if (ifzhi_L==1)
            {
                dir_luzhang=1;
            }
            else if (ifzhi_R==1)
            {
                dir_luzhang=-1;
            }
        }
        // ips200.show_float(150,16*19,lukuan,3,3); 
    }

    if (luzhang==1)
    {
        luzhangjuli+=Measure_Speedl-Measure_Speedr;
        // ips200.show_int(50,16*18,luzhangjuli,4); 
        if (luzhangjuli>=12000)
        {
            luzhang=0;
            luzhangjuli=0;
            dir_luzhang=0;
        }
    }

}
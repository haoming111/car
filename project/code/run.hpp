#ifndef _RUN_HPP_
#define _RUN_HPP_

#include "zf_common_headfile.hpp"

#define PI                  ( 3.1415926535898 )

void search_bianxian(void);
void zhongxianhecheng(void);
void xunxain_celue(void);
float get_angle(void);
void if_shizi(void);
void seek_up(uint8 kind);
void if_luzhang(void);

extern uint8 yuzhi;

extern bool ifzhi_L,ifzhi_R;
extern int ang_numL,ang_numR;
extern int shizi;
extern bool lost_l;
extern bool lost_r;
extern int L_count, R_count; 
extern float L_edge[100][2],R_edge[100][2];
extern float Lout_edge[100][2];     //左边界结构体
extern float Luse_edge[100][2];
extern float Rout_edge[100][2];     //左边界结构体
extern float Ruse_edge[100][2];
extern float ang_L[100],ang_R[100];     
extern int Lout_count,Rout_count; 

extern float Bu_L_edge[100][2];     //补线左边界结构体                        
extern float Bu_Lout_edge[100][2];     //补线边界结构体
extern float Bu_Luse_edge[100][2];
extern float Bu_R_edge[100][2];    //右边界结构体
extern float Bu_Rout_edge[100][2];     //左边界结构体
extern float Bu_Ruse_edge[100][2];
extern int Bu_L_count,Bu_R_count,Bu_Lout_count,Bu_Rout_count;
extern float Bu_ang_L[100],Bu_ang_R[100];  
extern int Bu_ang_numL,Bu_ang_numR;
extern bool BU_star_flag;

extern int luzhang;

#endif


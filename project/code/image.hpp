#ifndef _IMAGE_HPP_
#define _IMAGE_HPP_

#include "zf_common_headfile.hpp"

uint8 otsuThreshold(uint8 *image);
void get_highest(void);
void get_BLY(float L_line[][2],int *Lnum,float R_line[][2],int *Rnum,int len ,uint8_t L_Bu , uint8_t R_Bu);
void get_mid_L(float line[][2],int num,float mid[][2],int *num2,int approx_num);
void get_mid_R(float line[][2],int num,float mid[][2],int *num2,int approx_num);
int clip(int x, int low, int up);
float fclip(float x, float low, float up);
// void nitoushi(void);
void touxian(float line[][2],int *num);
void blur_points(float img_in[][2], float imag_out[][2], int num, int kernel);
void resample_points2(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist);
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist);
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist);
void nms_angle(float angle_in[], int num, float angle_out[], int kernel);
void guai_mum(float Edge[][2], int num, float ang_out[],int *posi,bool *if_zhi);
uint8 add_three_point_bezier(uint8 location1[2], uint8 location2[2], uint8 location3[2],
                             float pts[][2], uint8 add_location, uint8 dist);
uint8 Arry_roll(float pts_in[][2], float pts_out[][2], uint8 location1, uint8 location2, uint8 step, uint8 step_Max);
float SquareRootFloat(float number);
// uint8 Arry_Filter_2(uint8 pts_in[][2],uint8 step);
uint8 cbh(int x,int y,int k);

#endif
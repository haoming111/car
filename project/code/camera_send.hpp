#ifndef _CAMERA_SEND_HPP_
#define _CAMERA_SEND_HPP_

#include "zf_common_headfile.hpp"

#define SERVER_IP "192.168.31.211"// 另外一端的IP地址
#define PORT 8086// 端口号
#define BOUNDARY_NUM                (UVC_HEIGHT * 4 / 2)  // 边界点数量，适配TYPE3回弯场景，点数大于图像高度

void camera_send_init(void);
void Image_Get(void);
void camera_init(void);
void show_image(uint8 show_flag);

extern uint8 image_use[UVC_HEIGHT][UVC_WIDTH];
extern uint16* rgb_image;
#endif
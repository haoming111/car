#ifndef _KEY_HPP_
#define _KEY_HPP_

#include "zf_common_headfile.hpp"

#define KEY_1_PATH        ZF_GPIO_KEY_1
#define KEY_2_PATH        ZF_GPIO_KEY_2
#define KEY_3_PATH        ZF_GPIO_KEY_3
#define KEY_4_PATH        ZF_GPIO_KEY_4

struct keys
{
	bool key_sta;
	bool sin_flag;
	bool long_flag;
	unsigned char key_jude;
	unsigned int key_time;
};

void key_scanf(void);
void key_event_handler(void);

extern volatile bool key_scan_flag ;  
extern bool car_star;
extern zf_driver_gpio  beep;
#endif
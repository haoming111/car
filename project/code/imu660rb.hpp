#ifndef _IMU660RB_HPP_
#define _IMU660RB_HPP_

#include "zf_common_headfile.hpp"

void imu_init(void);
void get_raw(void);
void imu660rb_init(void);

extern float Measure_gyro;

#endif
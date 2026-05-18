#include "imu660rb.hpp"

zf_device_imu imu_dev;

int16 imu_acc_x,imu_acc_y,imu_acc_z;
int16 imu_gyro_x,imu_gyro_y,imu_gyro_z;
int16 imu_mag_x,imu_mag_y,imu_mag_z;
void imu_init()
{
    // IMU传感器初始化，自动检测挂载的IMU型号并完成底层配置
    imu_dev.init();

    // 识别IMU传感器型号并打印串口信息
    if(DEV_IMU660RA == imu_dev.imu_type)
    {
        printf("IMU DEV IS IMU660RA\r\n");
    }
    else if(DEV_IMU660RB == imu_dev.imu_type)
    {
        printf("IMU DEV IS IMU660RB\r\n");
    }
    else if(DEV_IMU963RA == imu_dev.imu_type)
    {
        printf("IMU DEV IS IMU963RA\r\n");
    }
    else
    {
        // 未检测到有效IMU设备，打印错误信息并退出程序
        printf("NO FIND IMU DEV\r\n");
        exit(-1);
    }

}


//零偏校准
bool flag_calib_failed=1;
uint8 imu_int_i=0;
int16 gyro_cur[3]={0}; int16 gyro_last[3]={0}; int16 gyro_int[3]={0}; int16 gyro_bias[3]={0};
void imu660rb_init()
{
    imu_init();
    while (flag_calib_failed) 
    {
        get_raw();
        system_delay_ms(2);
        if (imu_int_i < 200) 
        {
            imu_int_i++;                       // 1. 先计数
            gyro_cur[0] = imu_gyro_x;
            gyro_cur[1] = imu_gyro_y;
            gyro_cur[2] = imu_gyro_z;

            if (imu_int_i == 1) 
            {                              // 第 1 次保存上一帧
                gyro_last[0] = gyro_cur[0];
                gyro_last[1] = gyro_cur[1];
                gyro_last[2] = gyro_cur[2];
            }

            for (uint8 j = 0; j < 3; j++) 
            {
                gyro_int[j]  += func_abs(gyro_cur[j] - gyro_last[j]);
                gyro_bias[j] += gyro_cur[j];
                gyro_last[j]  = gyro_cur[j];
            }
        } 
        else 
        {                               // 200 次已满
            if (gyro_int[0] > 500 || gyro_int[1] > 500 || gyro_int[2] > 500) 
            {
                imu_int_i = 0;
                for (uint8 j = 0; j < 3; j++) 
                {
                    gyro_bias[j] = 0;
                    gyro_int[j]  = 0;
                }
                /* 保持 flag_calib_failed = 1，重新来 */
            } 
            else 
            {
                if (imu_int_i == 200) 
                {        // 只除一次
                    gyro_bias[0] *= 0.005f;
                    gyro_bias[1] *= 0.005f;
                    gyro_bias[2] *= 0.005f;
                    imu_int_i = 201;           // 防止重复除
                }
                flag_calib_failed = 0;         // 校准成功，退出循环
            }
        }
    }
    printf("IMU_OK\r\n");
}

float Measure_gyro,Measure_gyro_least;
float gyro_add=0; //滤波后的角速度
void get_raw()
{
    imu_gyro_x = imu_dev.get_gyro_x()/10;
    imu_gyro_y = imu_dev.get_gyro_y()/10;//俯仰角
    imu_gyro_z = imu_dev.get_gyro_z()/10;//偏航角

    //一阶互补滤波
    Measure_gyro_least=imu_gyro_z-gyro_bias[2];
    Measure_gyro *= 0.2;
    Measure_gyro += 0.8*Measure_gyro_least;
}
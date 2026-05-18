#include "tof.hpp"

zf_device_dl1x dl1x_dev;                  // DL1X设备对象
enum dl1x_device_type_enum dl1x_dev_type;  // DL1X设备类型
zf_driver_pit dl1x_pit_timer;             // PIT定时器对象（用于100ms定时采集）
volatile int16 dl1x_distance_raw = 0;      // 定时采集的距离原始数据（volatile防止编译器优化）

void tof_init()
{
        // DL1X传感器初始化
    dl1x_dev_type = dl1x_dev.init();
    // 判断DL1X初始化结果
    if(NO_FIND_DEVICE == dl1x_dev_type)
    {
        printf("错误：未找到DL1X设备，请检查硬件连接！\r\n");
        // return -1;
    }
    else
    {
        printf("DL1X 初始化成功！识别到：");
        if(ZF_DEVICE_DL1A == dl1x_dev_type) printf("ZF_DEVICE_DL1A\r\n");
        else if(ZF_DEVICE_DL1B == dl1x_dev_type) printf("ZF_DEVICE_DL1B\r\n");
    }
}
int get_tof()
{
    // 仅读取DL1X距离数据，存入全局变量（主循环负责打印，避免回调内耗时）
    if(NO_FIND_DEVICE != dl1x_dev_type)  // 仅当设备初始化成功时读取
    {
        dl1x_distance_raw = dl1x_dev.get_distance();
        return dl1x_distance_raw;
    }
}
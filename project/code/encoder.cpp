#include "encoder.hpp"

// 创建编码器对象，传入文件路径
zf_driver_encoder encoder_dir_1(ENCODER_DIR_1_PATH);
zf_driver_encoder encoder_dir_2(ENCODER_DIR_2_PATH);

int16 speed_L,speed_R;
float Measure_Speedl_least=0,Measure_Speedr_least=0;
float Measure_Speedl=0,Measure_Speedr=0; //滤波后的转速
void get_encoder()
{
    speed_L = encoder_dir_1.get_count();
    speed_R = encoder_dir_2.get_count();

    encoder_dir_1.clear_count();
    encoder_dir_2.clear_count();
    
    // //一阶互补滤波
    Measure_Speedl_least=speed_L;
    Measure_Speedr_least=speed_R;
    Measure_Speedl *= 0.2;
    Measure_Speedr *= 0.2;
    Measure_Speedl += 0.8*Measure_Speedl_least;
    Measure_Speedr += 0.8*Measure_Speedr_least;
}

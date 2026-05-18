#include "camera_send.hpp"
#include "motor.hpp"

zf_device_uvc uvc_dev;          // 定义UVC免驱摄像头的设备对象，用于摄像头的所有操作
zf_driver_tcp_client tcp_client_dev;  // 定义TCP客户端设备对象，用于建立TCP连接和网络数据收发

// ====================== TCP收发数据包装函数 ======================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介 TCP发送数据 全局包装函数
// 参数说明 buf 要发送的数据缓冲区指针, len 要发送的数据字节长度
// 返回参数 uint32 实际成功发送的字节数
// 使用示例 供seekfree_assistant_interface_init调用，无需手动调用
// 备注信息 封装tcp_client_dev.send_data成员函数，适配普通函数指针格式要求
//-------------------------------------------------------------------------------------------------------------------
uint32 tcp_send_wrap(const uint8 *buf, uint32 len)
{
    return tcp_client_dev.send_data(buf, len);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 TCP接收数据 全局包装函数
// 参数说明 buf 接收数据的缓冲区指针, len 最大可接收的字节长度
// 返回参数 uint32 实际成功接收的字节数
// 使用示例 供seekfree_assistant_interface_init调用，无需手动调用
// 备注信息 封装tcp_client_dev.read_data成员函数，适配普通函数指针格式要求
//-------------------------------------------------------------------------------------------------------------------
uint32 tcp_read_wrap( uint8 *buf, uint32 len)
{
    return tcp_client_dev.read_data(buf, len);
}


//包含三条边线信息，边界信息含有横纵轴坐标，意味着可以指定每个点的横纵坐标，边线的数量也可以大于或者小于图像的高度，通常来说边线数量大于图像的高度，一般是搜线算法能找出回弯的情况
uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];
// uint8 image_copy[UVC_HEIGHT][UVC_WIDTH];
uint8 image_use[UVC_HEIGHT][UVC_WIDTH];
uint8 image_see[UVC_HEIGHT][UVC_WIDTH];
void camera_send_init()
{
    // ====================== 第一步：初始化TCP客户端，建立网络连接 ======================
    if(tcp_client_dev.init(SERVER_IP, PORT) == 0)
    {
        printf("tcp_client ok\r\n");    // TCP连接成功，串口打印成功日志
    }
    else
    {
        printf("tcp_client error\r\n"); // TCP连接失败，串口打印错误日志
        exit(-1);                      // 连接失败直接退出程序，不执行后续流程
    }

    // ====================== 第二步：初始化逐飞助手通信接口 ======================
    // 实现上位机和下位机的TCP数据交互
    seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);

    // ====================== 第三步：根据边界类型配置图像与巡线边界信息 ======================
     seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_see[0], UVC_WIDTH, UVC_HEIGHT);
    // 配置完整XY轴巡线边界信息
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);

}

uint8* rgay_image;                    // 灰度图像数据指针，指向摄像头采集到的灰度图像缓冲区首地址
uint16* rgb_image;                     // 定义RGB图像数据指针，用于存放摄像头采集到的RGB图像首地址
void Image_Get()
{
    // 阻塞等待摄像头完成新一帧图像的采集和刷新
    if(uvc_dev.wait_image_refresh() < 0)
    {
        motor(0,0);
        exit(0);   // 摄像头采集异常，直接退出程序，防止卡死
    }
    // 获取摄像头采集到的灰度图像数据缓冲区首地址
    rgay_image = uvc_dev.get_gray_image_ptr();

    //彩色图传
    // // 获取摄像头采集到的RGB565格式图像数据的首地址指针
    // rgb_image = uvc_dev.get_rgb_image_ptr();
    // 图像数据格式转换：高低位字节交换
    // 原因：摄像头采集的RGB565数据高低位顺序与上位机解析要求不一致，必须交换后才能正常显示彩色图像
    // for(uint32_t i = 0; i < UVC_WIDTH * UVC_HEIGHT; i++)
    // {
    // ((uint16_t *)image_copy[0])[i] = (((uint16_t *)rgb_image)[i] & 0xFF) <<8 | (((uint16_t *)rgb_image)[i] & 0xFF00) >>8;
    // }

    // 获取摄像头采集到的RGB图像数据缓冲区首地址指针
    // RGB图像数据格式为uint16位宽（565格式：5位红+6位绿+5位蓝）
    rgb_image = uvc_dev.get_rgb_image_ptr();
    
    // 判空：确保图像指针有效，避免空指针操作导致程序崩溃
    if(NULL != rgb_image)
    {
        // 在屏幕的(0,0)坐标起点，显示RGB图像
        // 显示尺寸为摄像头采集的标准宽高 UVC_WIDTH * UVC_HEIGHT
        // ips200.displayimage_rgb565(rgb_image, UVC_WIDTH, UVC_HEIGHT);
    }

    memcpy(image_use[0], rgay_image, UVC_WIDTH*UVC_HEIGHT);
    // memcpy(image_see[0], rgay_image, UVC_WIDTH*UVC_HEIGHT);
   
}

void camera_init()
{
    if(uvc_dev.init(UVC_PATH) < 0)// UVC免驱摄像头初始化，传入USB摄像头设备路径
    {
        std::cerr << "摄像头初始化失败！请检查设备节点或摄像头连接" << std::endl;
        exit(-1);
    }
}

extern zf_device_ips200 ips200;
// 显示图像到屏幕上  
void show_image(uint8 show_flag)// 显示模式切换标志位：0-显示灰度图像，1-显示RGB彩色图像
{
    if(show_flag) // 标志位为1，显示RGB彩色图像
    {
        // 获取摄像头采集到的RGB图像数据缓冲区首地址指针
        // RGB图像数据格式为uint16位宽（565格式：5位红+6位绿+5位蓝）
        uint16* rgb_image = uvc_dev.get_rgb_image_ptr();
        // 判空：确保图像指针有效，避免空指针操作导致程序崩溃
        if(NULL != rgb_image)
        {
            ips200.displayimage_rgb565(rgb_image, UVC_WIDTH, UVC_HEIGHT);
        }
    }
    else // 标志位为0，显示灰度图像
    {
        // 获取摄像头采集到的灰度图像数据缓冲区首地址指针
        // 灰度图像数据格式为uint8位宽（0-255表示亮度等级，0最暗，255最亮）
        uint8* gray_image = uvc_dev.get_gray_image_ptr();
        // 判空：确保图像指针有效
        if(NULL != gray_image)
        {
            ips200.displayimage_gray(gray_image, UVC_WIDTH, UVC_HEIGHT);
        }
    }
}
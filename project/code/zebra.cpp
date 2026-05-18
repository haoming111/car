#include "zebra.hpp"
#include "run.hpp"
#include "ring.hpp"
#include "camera_send.hpp"

bool zebra = false;

bool judge_time(uint8 y,int target);
/**
 * @brief 斑马线检测函数，根据速度和阈值判定是否检测到斑马线
 * @param target 边缘变化次数阈值，当变化次数大于该值时，认为检测到斑马线
 */
bool zebra_judge(int target,bool state)
{
    for(int i = Start_high; i > 80; i--) 
    {
        if(judge_time(i, target))
        {
            if(state) zebra = true;
            return true;
        } 
    }
    return false;
}

/**
 * @brief 判断指定位置是否满足斑马线判定条件
 * @param position 图像垂直位置（对应边界点的索引）
 * @param target 边缘变化次数阈值
 * @return 如果满足条件返回 true，否则返回 false
 */
bool judge_time(uint8 y,int target)
{
    uint8 count = 0;
    bool now_state = false, last_state = false;
    
    // 设置初始状态为第一个像素的状态
    if(image_use[y][0] > yuzhi) 
    {
        last_state = true;
    } 
    else 
    {
        last_state = false;
    }
    
    // 从第二个像素开始计算边缘变化次数
    for(uint8 i = 1; i < 159; i++) 
    {
        if(image_use[y][i] > yuzhi) 
        {
            now_state = true;
        }
        else 
        {
            now_state = false;
        }
        
        // 检测到状态变化时计数加1
        if(now_state != last_state) 
        {
            count++;
        }
        last_state = now_state;
    }
    return (count > target);
}

uint8 stop_check_count = 0;
bool stop_check(uint8 threshold)
{
    int sum = 0;
    for(uint8 i = 0; i < UVC_WIDTH; i++)
    {
        if(image_use[Start_high][i] < yuzhi)
        {
            sum++;
        }
    }

    if(sum > threshold)
    stop_check_count++;
    else 
    stop_check_count = 0;

    if(stop_check_count > 3)
    return true;
    else return false;
}

uint16 zebra_state=0;
extern zf_driver_gpio  beep;
extern int red_state;
int zebra_num;
int zebra_use=0;
void ifzebra()
{
    switch (zebra_state)
    {
        case 0:
            if(ring_ctrl.kind == no_ring &&shizi==0 &&luzhang==0&&red_state==0&&zebra_state==0)
            zebra_state=zebra_judge(10,1);
        break;
    
        case 1:
            if(zebra_judge(10,1))
            {
                zebra_num++;
            }
            else 
                zebra_num--;
            if(zebra_num>3)
            {
                zebra_num=0;
                zebra_state=2;
                beep.set_level(1);
            }
            else if(zebra_num<-1)
            {
                zebra_num=0;
                zebra_state=0;
            }
        break;

        case 2:
            zebra_num++;
            bool zebra_flag=zebra_judge(10,1);
            if(zebra_flag==0 && zebra_num>20)
            {
                beep.set_level(0);
                zebra_num=0;
                zebra_use++;
                zebra_state=0;
                // printf("%d\n",zebra_use);
            }
        break;            
    }
}
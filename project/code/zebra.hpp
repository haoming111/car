#ifndef __Zebra_HPP__
#define __Zebra_HPP__

#include "zf_common_headfile.hpp"

/**
 * @brief 斑马线判定结果，true 表示检测到斑马线
 */
extern bool zebra;
extern uint16 zebra_state;
/**
 * @brief 斑马线检测函数，根据阈值判定是否检测到斑马线
 * @param target 边缘变化次数阈值，当变化次数大于该值时，认为检测到斑马线
 */
bool zebra_judge(int target,bool state);
bool stop_check(uint8 threshold);
void ifzebra(void);

#endif

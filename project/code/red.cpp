#include "red.hpp"

#define RED_THRESHOLD_RATIO 0.1   // 0.6  红色占比阈值（60%）
#define RED_DIFF_RG 40           // R-G差值阈值
#define RED_DIFF_RB 40           // R-B差值阈值
#define RED_MIN_R   100          // 最小红色强度

static inline int is_red_pixel(uint16_t pixel)
{
    // 提取RGB565
    uint8_t r = (pixel >> 11) & 0x1F;
    uint8_t g = (pixel >> 5)  & 0x3F;
    uint8_t b = pixel & 0x1F;

    // 放大到 0~255
    r <<= 3;
    g <<= 2;
    b <<= 3;

    // 判定红色（核心条件）
    if (r > RED_MIN_R &&(r - g) > RED_DIFF_RG &&(r - b) > RED_DIFF_RB)
    {
        return 1;
    }
    return 0;
}

bool is_red_region(uint16_t* img ,int width,int height,int x_start,int x_end,int y_start,int y_end)
{
    if (!img) return false;

    // 边界保护（非常重要）
    if (x_start < 0) x_start = 0;
    if (y_start < 0) y_start = 0;
    if (x_end >= width) x_end = width - 1;
    if (y_end >= height) y_end = height - 1;

    int total = 0;
    int red_count = 0;

    for (int y = y_start; y <= y_end; y++)
    {
        for (int x = x_start; x <= x_end; x++)
        {
            uint16_t pixel = img[y * width + x];

            total++;

            if (is_red_pixel(pixel))
            {
                red_count++;
            }
        }
    }

    if (total == 0) return false;

    float ratio = (float)red_count / total;

    return (ratio > RED_THRESHOLD_RATIO);
}
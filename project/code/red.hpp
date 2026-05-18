#ifndef _RED_HPP_
#define _RED_HPP_

#include "zf_common_headfile.hpp"

bool is_red_region(uint16_t* img,
                   int width,
                   int height,
                   int x_start,
                   int x_end,
                   int y_start,
                   int y_end);

#endif
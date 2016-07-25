#include "../include/point_xy_mixed.h"

cos_lib::point_xy_mixed::point_xy_mixed() : point_xy_rgb()
{
}

cos_lib::point_xy_mixed::point_xy_mixed(uint32_t rgb, unsigned short greyscale) : point_xy_rgb(rgb)
{
    this->_greyscale = greyscale;
}

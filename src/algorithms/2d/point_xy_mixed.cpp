#include "point_xy_mixed.h"

cloud_object_segmentation::point_xy_mixed::point_xy_mixed() : point_xy_rgb()
{
}

cloud_object_segmentation::point_xy_mixed::point_xy_mixed(uint32_t rgb, unsigned short greyscale) : point_xy_rgb(rgb)
{
    this->_greyscale = greyscale;
}

#include "point_xy_rgb.h"

cloud_object_segmentation::point_xy_rgb::point_xy_rgb()
{
}

cloud_object_segmentation::point_xy_rgb::point_xy_rgb(uint32_t rgb)
{
    this->_rgb = rgb;
}

#include "point_xy_rgb.h"

namespace ns_cos = cloud_object_segmentation;

ns_cos::point_xy_rgb::point_xy_rgb()
{
}

ns_cos::point_xy_rgb::point_xy_rgb(uint32_t rgb)
{
    this->_rgb = rgb;
}

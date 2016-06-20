#include "point_xy_mixed.h"

namespace ns_cos = cloud_object_segmentation;

ns_cos::point_xy_mixed::point_xy_mixed() : point_xy_rgb()
{
}

ns_cos::point_xy_mixed::point_xy_mixed(uint32_t rgb, unsigned short greyscale) : point_xy_rgb(rgb)
{
    this->_greyscale = greyscale;
}

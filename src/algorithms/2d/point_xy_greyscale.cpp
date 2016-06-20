#include "point_xy_greyscale.h"

namespace ns_cos = cloud_object_segmentation;

ns_cos::point_xy_greyscale::point_xy_greyscale()
{
}

ns_cos::point_xy_greyscale::point_xy_greyscale(unsigned short greyscale)
{
    this->greyscale(greyscale);
}

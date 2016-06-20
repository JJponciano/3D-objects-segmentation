#include "image.h"

namespace ns_cos = cloud_object_segmentation;

ns_cos::image::image(size_t width, size_t height)
{
    this->_width = width;
    this->_height = height;
}

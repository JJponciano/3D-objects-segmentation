#include "vector3.h"

namespace ns_cos = cloud_object_segmentation;

ns_cos::aux::vector3::vector3()
{

}

ns_cos::aux::vector3::vector3(float x, float y, float z)
{
    this->x(x);
    this->y(y);
    this->z(z);
}


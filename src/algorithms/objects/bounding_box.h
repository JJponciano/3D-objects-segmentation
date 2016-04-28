#include <vector>
#include <pcl/point_types.h>

#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

class bounding_box
{
public:
    bounding_box();

private:
    std::vector<pcl::PointXYZRGB*> points;
    std::vector<bounding_box> boxes;
};

#endif // BOUNDING_BOX_H

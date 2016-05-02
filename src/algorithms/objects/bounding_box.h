#include <vector>
#include "pointbool.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

class bounding_box
{
public:
    bounding_box(pcl::PointCloud<clstr::PointBool>::Ptr cloud);
    bounding_box(clstr::PointBool* A,clstr::PointBool* B,clstr::PointBool* C,clstr::PointBool* D,clstr::PointBool* E,clstr::PointBool* F,clstr::PointBool* G,clstr::PointBool* H);

private:
    clstr::PointBool* A;
    clstr::PointBool* B;
    clstr::PointBool* C;
    clstr::PointBool* D;
    clstr::PointBool* E;
    clstr::PointBool* F;
    clstr::PointBool* G;
    clstr::PointBool* H;
    std::vector<pcl::PointXYZRGB*> points;
    std::vector<bounding_box> boxes;

    std::vector<bounding_box*> divideBox();
};

#endif // BOUNDING_BOX_H

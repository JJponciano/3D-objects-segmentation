#include <vector>
#include "point_clstr.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

class bounding_box
{
public:
    bounding_box(pcl::PointCloud<clstr::point_clstr>::Ptr cloud);
    bounding_box(clstr::point_clstr* A,clstr::point_clstr* B,clstr::point_clstr* C,clstr::point_clstr* D,clstr::point_clstr* E,clstr::point_clstr* F,clstr::point_clstr* G,clstr::point_clstr* H);
    std::vector<bounding_box*> divideBox();
    void addPointIntoBox(clstr::point_clstr* point);
    int getNbOfPoints() { return this->points.size(); }
    std::vector<clstr::point_clstr*> getVertices();
    void deleteBox();
private:
    clstr::point_clstr* A;
    clstr::point_clstr* B;
    clstr::point_clstr* C;
    clstr::point_clstr* D;
    clstr::point_clstr* E;
    clstr::point_clstr* F;
    clstr::point_clstr* G;
    clstr::point_clstr* H;
    std::vector<clstr::point_clstr*> points;
    std::vector<bounding_box*> boxes;

};

#endif // BOUNDING_BOX_H

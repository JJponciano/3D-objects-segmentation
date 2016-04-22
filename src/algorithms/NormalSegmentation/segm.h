#ifndef SEGM_H
#define SEGM_H

#include "geom_op.h";

namespace segm
{
    // function that groups the points of a cloud by their value
    // each category is represented by a vector
    // the categories are added to a vector which is returned at the end
    // cloud_normals is the parameter that contains the pairs of points and normals
    std::vector<std::vector<pcl::PointXYZRGB *> *> pts_regrp(std::vector<std::pair<pcl::PointXYZRGB *, std::vector<float>>> cloud_normals, float eps);

    // procedure that colors every point of the graph, the criterion being the category of the point
    // gr_pts is a 'set' of all the different categories categories
    void pts_colsegm(std::vector<std::vector<pcl::PointXYZRGB *> *> gr_pts);
}

#endif // SEGM_H

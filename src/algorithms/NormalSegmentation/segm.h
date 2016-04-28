/**
  * @brief library that contains function allowing the segmentation of a point cloud by category of points and colors
  * @author Vlad-Adrian Moglan
  */

#ifndef SEGM_H
#define SEGM_H

#include "geom_op.h";

namespace segm
{
    /**
     * @brief pts_regrp regroups points in function of their respective spherical coordinates
     * @param spherical_coords is a vector of pairs formed by one point and one vector containing the point's spherical coordinates
     * @param eps is the degree of precision used for comparing spherical coordinates
     * @return an array of categories created in function of the points' coordinates
     */
    std::vector<std::vector<pcl::PointXYZRGB *> *> pts_regrp(std::vector<std::pair<pcl::PointXYZRGB *, std::vector<float>>> spherical_coords, float eps);

    /**
     * @brief pts_colsegm colors points in function of their category
     * @param gr_pts is an array of categories containing points
     */
    void pts_colsegm(std::vector<std::vector<pcl::PointXYZRGB *> *> gr_pts);
}

#endif // SEGM_H

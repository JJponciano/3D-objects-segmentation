#ifndef EFF_NORM_EST_H
#define EFF_NORM_EST_H

#include "normal_estimation.h"
#include "cloud_manip.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief eff_norm_est is an efficient way of coloring a cloud by its estimated normal vectors
 * @details it uses multi-threading and cloud segmentation
 * @param pt_cl is the point cloud to find the normals of
 * @param radius is the range in which a k-d tree will find the neighbours of a point
 * @param max_neighbs is the maximum number of neighbours returned by the k-d tree radius search
 * @param y_scale is scaling parameter for the y axis of a widop cloud
 * @return the colored cloud
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr eff_norm_est(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl, float radius, int max_neighbs, float y_scale);

#endif // EFF_NORM_EST_H

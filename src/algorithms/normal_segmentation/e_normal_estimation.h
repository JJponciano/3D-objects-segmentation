#ifndef EFF_NORM_EST_H
#define EFF_NORM_EST_H

#include "normal_estimation.h"
#include "../cloud_manip/cloud_manip.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief e_normal_estimation an efficient way of coloring a cloud by its estimated normal vectors
 * @details it uses multi-threading and cloud segmentation
 * @param cloud is the point cloud to find the normals of
 * @param radius is the range in which a k-d tree will find the neighbours of a point
 * @param max_neighbs is the maximum number of neighbours returned by the k-d tree radius search
 * @param x_scale is a scaling parameter for the x axis of a cloud
 * @param y_scale is a scaling parameter for the y axis of a cloud
 * @param z_scale is a scaling parameter for the z axis of a cloud
 * @param precision is the precision for float comparison
 * @return the colored cloud
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr e_normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                    float radius,
                                                    int max_neighbs,
                                                    float x_scale,
                                                    float y_scale,
                                                    float z_scale,
                                                    float precision);

#endif // EFF_NORM_EST_H

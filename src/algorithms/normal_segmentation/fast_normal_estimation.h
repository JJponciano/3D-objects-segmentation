#ifndef EFF_NORM_EST_H
#define EFF_NORM_EST_H

#include "normal_estimation.h"
#include "../tools/cloud_manip.h"
#include "../except/invalid_cloud_pointer.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QString>

/**
 * @brief fast_normal_estimation an efficient way of coloring a cloud by its estimated normal vectors
 * @details it uses multi-threading and cloud segmentation
 * @param cloud_ptr is a pointer to the point cloud to find the normals of
 * @param radius is the range in which a k-d tree will find the neighbours of a point
 * @param max_neighbs is the maximum number of neighbours returned by the k-d tree radius search
 * @param x_scale is a scaling parameter for the x axis of a cloud
 * @param y_scale is a scaling parameter for the y axis of a cloud
 * @param z_scale is a scaling parameter for the z axis of a cloud
 * @param max_fragment_depth is the maximum depth a cloud fragment can measure; it is not scaled
 * @return the colored cloud
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr fast_normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, int max_neighbs,
                                                    float radius, float x_scale, float y_scale, float z_scale, float max_fragment_depth);

#endif // EFF_NORM_EST_H

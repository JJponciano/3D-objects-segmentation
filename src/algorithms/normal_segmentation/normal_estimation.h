/**
  * @author Jean-Jacques Ponciano (algorithm), Vlad-Adrian Moglan (code)
  */

#ifndef NORMAL_ESTIMATION_H
#define NORMAL_ESTIMATION_H

#include "../tools/aux_op.h"

namespace cloud_object_segmentation
{
    namespace normal_segmentation
    {
        /**
         * @brief estimate_normals is a function that estimates the normal vectors of a point cloud
         * @param cloud_ptr is a pointer to the point cloud to estimates the normal vectors of
         * @param radius defines the range in which the k-d tree of cloud will look for the closest neighbours of a given point of the cloud
         * @param max_neighbs is the maximum number of neighbours the kd-tree search function should return
         */
        void estimate_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, float radius, int max_neighbs);

        /**
         * @brief estimate_normals is a function that estimates the normals of the parameter cloud using the standard pcl library
         * @param cloud_ptr is a pointer to the point cloud to find the normals of
         * @throw std::invalid_argument if cloud_ptr is nullptr
         */
        void estimate_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);
    }
}

#endif // NORMAL_ESTIMATION_H

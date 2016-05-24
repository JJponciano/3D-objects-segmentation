/**
  * @brief a library containing functions that allow vector operations as well as auxilliary functions for other calculations
  * @author Vlad-Adrian Moglan
  */

#ifndef GEOM_OP_H
#define GEOM_OP_H

#include "vector3.h"
#include "../except/invalid_cloud_pointer.h"

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <boost/lexical_cast.hpp>

#include <QString>

namespace vector3_operations
{
    /**
     * @brief vect_2pts creates a vector using two points
     * @param pt_1 is the point at the arrow of the vector
     * @param pt_2 is the point at the 'root' of the vector
     * @return the vector that has been created using 2 points
     */
    vector3 vect_2pts(pcl::PointXYZRGB pt_1, pcl::PointXYZRGB pt_2);

    /**
     * @brief cross_product calculates the cross product of two vectors
     * @details the cross product of two vectors only has sense in R^3
     * @param vect_1 is the first factor of the product
     * @param vect_2 is the second factor of the product
     * @return the vector result of the operation
     */
    vector3 cross_product(vector3 vect_1, vector3 vect_2);

    /**
     * @brief inverse inverses the coordinates of a vector
     * @param vect is the vector to be inversed
     * @return the inversed vector
     */
    vector3 inverse(vector3 vect);

    /**
     * @brief translate_origin translates a vector into the origin
     * @param x_1 is the x coordinate of the point at the root of the vector
     * @param y_1 is the y coordinate of the point at the root of the vector
     * @param z_1 is the z coordinate of the point at the root of the vector
     * @param x_2 is the x coordinate of the point at the arrow of the vector
     * @param y_2 is the y coordinate of the point at the arrow of the vector
     * @param z_2 is the z coordinate of the point at the arrow of the vector
     * @return the vector as a result of the translation
     */
    vector3 translate_origin(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2);

    /**
     * @brief vect_avg calculates the average of the vectors within an array of vectors
     * @param vectors is the array of the vectors we need the average of
     * @return the vector resulted the average of the vectors found within the parameter
     */
    vector3 vect_avg(std::vector<vector3> vectors);

    /**
     * @brief normalize_normal maps the values of the coordinates of a normal between 0 and 1
     * @param normal is the normal to be mapped
     * @return the mapped (normalized) normal
     */
    vector3 normalize_normal(vector3 normal);

    /**
     * @brief pcl_normal_estimation is a function that estimates the normals of the parameter cloud using the standard pcl library
     * @param cloud_ptr is a pointer to the point cloud to find the normals of
     * @throw std::invalid_argument if cloud_ptr is nullptr
     */
    void pcl_normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);
}

#endif // GEOM_OP_H

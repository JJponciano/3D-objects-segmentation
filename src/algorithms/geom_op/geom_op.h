/**
  * @brief a library containing functions that allow vector operations as well as auxilliary functions for other calculations
  * @author Vlad-Adrian Moglan
  */

#ifndef GEOM_OP_H
#define GEOM_OP_H

#include "vector3.h"

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <boost/lexical_cast.hpp>

namespace geom
{
    /**
     * namespace "vectors" contains vector related functions
     */
    namespace vectors
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
         * @param pt_cl is the point cloud to find the normals of
         */
        void pcl_normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl);
    }

    /**
     * namespace "aux" contains auxiliary functions
     */
    namespace aux
    {
        /**
         * @brief map maps a number found within a range to another range
         * @param x is the number to be mapped
         * @param in_min is minimum of the in range
         * @param in_max is the maximum of the in range
         * @param out_min is the minimum of the out range
         * @param out_max is the maximum of the out range
         * @return the mapped float
         */
        float map(float x,
                  float in_min, float in_max,
                  float out_min, float out_max) { return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

        /**
         * @brief float_avg calculates the average of float numbers
         * @param floats is the array of the floats to be averaged
         * @return the averaged of the floats found within the parameter array
         */
        float float_avg(std::vector<float> floats);

        /**
         * @brief spherical_coordinates calculates the spherical coordinates of the point at the arrow of an origin translated vector
         * @param vect the vector having its root at the origin
         * @return an array of 3 floats corresponding to the 3 spherical coordinates of a point (radius, inclination and azimuth)
         */
        std::vector<float> spherical_coords(vectors::vector3 vect);

        /**
         * @brief cmp_angles compares the inclination and azimuth angles (spherical coordinate system) of two points
         * @param coords_1 is an array containing the radius, inclination and azimuth of the first point
         * @param coords_2 is an array containing the radius, inclination and azimuth of the second point
         * @param precision is a float that defines the degree of precision of the comparison
         * @return true if the angles are equal within the boundries of epsilon
         */
        bool cmp_angles(std::vector<float> coords_1, std::vector<float> coords_2, float precision);

        /**
         * @brief cmp_floats compares
         * @param float_1
         * @param float_2
         * @param precision
         * @return true if the two parameter floats are equal within a certain range
         */
        bool cmp_floats(float float_1, float float_2, float precision);

        /**
         * @brief norm_to_rgb calculates the r, g and b values of a point in function of the coordinates of a given normal
         * @param pt is a pointer to the point to be colored
         * @param normal is the normal used for the calculation
         */
        void normal_to_rgb(pcl::PointXYZRGB *pt, geom::vectors::vector3 normal);

        /**
         * @brief abs_vector calculates the absolute values of the coordinates of a vector
         * @param vect is the vector to get the absolute values of
         * @return the resulting, absolute value coordinates vector
         */
        geom::vectors::vector3 abs_vector(geom::vectors::vector3 vect);

        // performs the euclidian distance calculation on two 3D and/or RGB points
        template<typename T>
        float euclidian_dist(T a_1, T b_1, T c_1,
                         T a_2, T b_2, T c_2)
        {
            return std::sqrt((float)(std::pow(a_1 - a_2, 2)
                                     + std::pow(b_1 - b_2, 2)
                                     + std::pow(c_1 - c_2, 2)));
        }
    }
}

#endif // GEOM_OP_H

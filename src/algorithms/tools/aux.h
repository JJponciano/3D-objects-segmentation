#ifndef AUX_H
#define AUX_H

#include "vector3.h"

#include <vector>

#include <pcl/point_types.h>

namespace aux
{
    /**
     * @brief set_precision sets the number of digits after the decimal
     * @param float_num is the number to be truncated
     * @param precision is the number of digits after the decimal
     * @throw std::logic_error when precision is negative, null or not a multiple of 10
     */
    float set_precision(float float_num, float precision);

    /**
     * @brief map maps a number found within a range to another range
     * @param x is the number to be mapped
     * @param in_min is minimum of the in range
     * @param in_max is the maximum of the in range
     * @param out_min is the minimum of the out range
     * @param out_max is the maximum of the out range
     * @return the mapped float
     */
    float map(float x, float in_min, float in_max, float out_min, float out_max);

    /**
     * @brief float_avg calculates the average of float numbers
     * @param floats is the array of the floats to be averaged
     * @return the averaged of the floats found within the parameter array
     */
    float float_avg(std::vector<float> floats);

    /**
     * @brief float_cmp compares two floats with a certain precision
     * @param float_1 is the first float of the comparison
     * @param float_2 is the second float of the comparison
     * @param precision is a float that defines the degree of precision of the comparison
     * @throw std::logic_error when precision is 0
     * @return true if the two parameter floats are equal within a certain range
     */
    bool float_cmp(float float_1, float float_2, float precision);

    /**
     * @brief spherical_coordinates calculates the spherical coordinates of the point at the arrow of an origin translated vector
     * @param vect the vector having its root at the origin
     * @return an array of 3 floats corresponding to the 3 spherical coordinates of a point (radius, inclination and azimuth)
     */
    std::vector<float> spherical_coords(vector3 vect);

    /**
     * @brief coords_cmp compares the inclination and azimuth angles (spherical coordinate system) of two points
     * @param coords_1 is an array containing the radius, inclination and azimuth of the first point
     * @param coords_2 is an array containing the radius, inclination and azimuth of the second point
     * @param precision is a float that defines the degree of precision of the comparison
     * @throw std::logic_error when precision is 0
     * @return true if the angles are equal within the boundries of epsilon
     */
    bool coord_cmp(std::vector<float> coords_1, std::vector<float> coords_2, float precision);

    /**
     * @brief norm_to_rgb calculates the r, g and b values of a point in function of the coordinates of a given normal
     * @param pt is a pointer to the point to be colored
     * @param normal is the normal used for the calculation
     */
    void normal_to_rgb(pcl::PointXYZRGB *pt, vector3 normal);

    /**
     * @brief vector_abs calculates the absolute values of the coordinates of a vector
     * @param vect is the vector to get the absolute values of
     * @return the resulting, absolute value coordinates vector
     */
    vector3 vector_abs(vector3 vect);

    /** @brief performs the euclidian distance calculation between two 3D points */
    template<typename T>
    float euclidian_dist(T a_1, T b_1, T c_1,
                     T a_2, T b_2, T c_2)
    {
        return std::sqrt((float)(std::pow(a_1 - a_2, 2)
                                 + std::pow(b_1 - b_2, 2)
                                 + std::pow(c_1 - c_2, 2)));
    }

}

#endif

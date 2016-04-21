#ifndef GEOM_OP_H
#define GEOM_OP_H
#define PI 3.1415927;

#include "vector3.h"

#include <map>
#include <iostream>
#include <sstream>
#include <fstream>
#include <ios>

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <boost/lexical_cast.hpp>


namespace geom
{
    namespace vectors
    {
        // function that calculates the cross product of two vector3s; limited to R^3
        //  vect1 and vect2 are the two vectors of the cross product
        vector3 *cross_product(vector3 vect1, vector3 vect2);

        // function that inverses the vector vect
        vector3 *inverse(vector3 vect);

        // function that translates the vector
        // vect1 is the vector to be translated using the values of vect2
        vector3 *translate_origin(float x1, float y1, float z1, float x2, float y2, float z2);

        // function that scales a vector (dot product)
        // vect1 is the vector to be scaled using the values in vect2
        vector3 *scale(vector3 vect1, vector3 vect2);

        // function that calculates the distance between two vectors vect1 and vect2
        float dist(vector3 vect1, vector3 vect2);

        // returns the averages of parameter vectors
        vector3 *vect_avg(std::vector<vector3> vectors);
    }

    namespace aux
    {
        // function that returns the average of a vector of floats
        float calc_avg(std::vector<float> floats);

        // function that calculates an angle given 3 points in a plane
        // x2 and y2 are the coordinates of the point that is the origin of the angle
        // x1 and y1 are the coordinates of the end point of one vector
        // x3 and y3 are the coordinates of the end point of the other vector
        float calc_angle3p(float x1, float y1, float x2, float y2, float x3, float y3);

        // function that calculates the spherical coordinates of a point using its cartesian coordinates
        std::vector<float> calc_sphcoord(vectors::vector3 vect);

        // function that compares two normal; returns true if the normal is similar enough
        bool cmp_norm(vectors::vector3 vect1, vectors::vector3 vect2);

        // compares the angles of two vectors
        bool cmp_angles(std::vector<float> vect1, std::vector<float> vect2, float epsilon);
    }



    // function that calculates the angles theta and phi of the normal calculated in each point
    // of the point cloud;
    // the function returns a dictionary containing the points and the values
    // of the angles of their respective normal; the values of the angles will be stored into a string for easily performing
    // comparisons between points;
    // the result of this function will serve as a manner of grouping points that are alike together;
    // the only parameter of this function is the point cloud
    std::vector<std::pair<pcl::PointXYZRGB *, std::vector<float>>> estim_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, float range);

    // pcl library that allows estimating the normals in the paramater cloud
    void pcl_estim_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);

}

#endif // GEOM_OP_H

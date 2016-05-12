#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../objects/bounding_box.h"
#include "../objects/point_clstr.h"

#ifndef BOUNDING_H
#define BOUNDING_H


class bounding
{
public:
    bounding();

    /**
     * @brief getCloudBoundings analyze a cloud to get only its external points
     * @param cloud a CLSTR point_clstr cloud which we want to find the boundings
     * @param iteration The number of time you want the algorithm to be used
     * @return a CLSTR point_clstr cloud only containing the external boundings points
     */
    static pcl::PointCloud<clstr::point_clstr>::Ptr getCloudBoundings(pcl::PointCloud<clstr::point_clstr>::Ptr cloud, int iteration = 100);
};

#endif // BOUNDING_H

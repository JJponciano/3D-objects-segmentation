#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../objects/bounding_box.h"
#include "../objects/pointbool.h"

#ifndef BOUNDING_H
#define BOUNDING_H


class bounding
{
public:
    bounding();

    /**
     * @brief getCloudBoundings analyze a cloud to get only its external points
     * @param cloud a CLSTR PointBool cloud which we want to find the boundings
     * @param iteration The number of time you want the algorithm to be used
     * @return a CLSTR PointBool cloud only containing the external boundings points
     */
    static pcl::PointCloud<clstr::PointBool>::Ptr getCloudBoundings(pcl::PointCloud<clstr::PointBool>::Ptr cloud, int ireration = 100);
};

#endif // BOUNDING_H

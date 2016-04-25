#include "cloud_manip.h"

void cloud_manip::widop_to_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl)
{
    // iterator
    pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it;

    // scaling cloud
    for (cloud_it = pt_cl->points.begin(); cloud_it < pt_cl->points.end(); cloud_it++)
        (*cloud_it).y /= Y_SCALE;
}

void cloud_manip::cloud_to_widop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl)
{
    // iterator
    pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it;

    // scaling cloud
    for (cloud_it = pt_cl->points.begin(); cloud_it < pt_cl->points.end(); cloud_it++)
        (*cloud_it).y *= Y_SCALE;
}

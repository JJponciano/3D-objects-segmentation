#include "normal_estimation.h"

void estim_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl, float radius, int max_neighbs)
{
    // kd-tree used for finding neighbours
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdt;

    // initializing tree
    kdt.setInputCloud(pt_cl);

    // cloud iterator
    pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it;

    for (cloud_it = pt_cl->points.begin(); cloud_it < pt_cl->points.end(); cloud_it++)
        geom::vectors::find_normal(pt_cl, cloud_it, kdt, radius, max_neighbs);
}

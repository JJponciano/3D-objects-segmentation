#include "test_lib.h"

void test_normal_estimation()
{
    // point cloud to be examined
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
\
    // loading cloud from file
    cloud = pcloud_io::load_cloud("perfectTable.txt");

    // estimating normals
    estim_normals(cloud, RANGE);
}

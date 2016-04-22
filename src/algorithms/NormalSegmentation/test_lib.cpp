#include "test_lib.h"

void test_normal_estimation()
{
    // point cloud to be examined
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
\
    // loading cloud from file
    cloud = pcloud_io::load_cloud("./table.txt");

    // estimating normals
    estim_normals(cloud, RADIUS);

    // writing cloud to file
    pcloud_io::cloud_txt("./table_rgbTest.txt", cloud);
}

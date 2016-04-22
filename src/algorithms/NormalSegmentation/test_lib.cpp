#include "test_lib.h"

void test_normal_estimation()
{
    std::cout << "> normal estimation function test start..." << std::endl;

    // point cloud to be examined
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
\
    // loading cloud from file
    cloud = pcloud_io::load_cloud("./table.txt", false);

    // estimating normals
    estim_normals(cloud, RADIUS);

    // writing cloud to file
    pcloud_io::cloud_to_txt("./table_rgbTest.txt", cloud);

    std::cout << "> done testing the normal estimation function." << std::endl;
}

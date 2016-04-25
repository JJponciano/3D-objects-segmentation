#include "test_lib.h"

void test_normal_estimation()
{
    std::cout << "> normal estimation function test start..." << std::endl;

    // point cloud to be examined
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::cout << "> loading cloud from file..." << std::endl;

    // loading cloud from file
    cloud = pcloud_io::load_cloud("../../../data/table.txt", false);

    std::cout << "> estimating normals..." << std::endl;

    // estimating normals
    estim_normals(cloud, RADIUS);

    std::cout << "> dones estimating normals..." << std::endl;

    std::cout << "> writing cloud to file..." << std::endl;

    // writing cloud to file
    pcloud_io::cloud_to_txt("./table_rgbTest.txt", cloud);

    std::cout << "> done testing the normal estimation function." << std::endl << std::endl;
}

void test_scaling()
{
    std::cout << "> scaling function test start..." << std::endl;

    // point cloud to be examined
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
\
    std::cout << "> loading cloud from file..." << std::endl;

    // loading cloud from file
    cloud = pcloud_io::load_cloud("../../../data/widop_test.txt", false);

    std::cout << "> scaling cloud..." << std::endl;

    // scaling cloud
    cloud_manip::widop_to_cloud(cloud);

    std::cout << "> done scaling the widop cloud;" << std::endl;

    std::cout << "> estimating normals..." << std::endl;

    // estimating normals
    estim_normals(cloud, RADIUS);

    std::cout << "> done estimating normals;" << std::endl;

    std::cout << "> descaling cloud..." << std::endl;

    // descaling cloud
    cloud_manip::cloud_to_widop(cloud);

    std::cout << "> done descaling the widop cloud;" << std::endl;

    std::cout << "> writing cloud to file..." << std::endl;

    // writing cloud to file
    pcloud_io::cloud_to_txt("../../../data/test_results/widop_testres.txt", cloud);

    std::cout << "> done testing the scaling function." << std::endl << std::endl;
}

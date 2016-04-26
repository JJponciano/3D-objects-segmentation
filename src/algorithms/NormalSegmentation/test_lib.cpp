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
    estim_normals(cloud, RADIUS, 100);

    std::cout << "> dones estimating normals..." << std::endl;

    std::cout << "> writing cloud to file..." << std::endl;

    // writing cloud to file
    pcloud_io::cloud_to_txt("./table_rgbTest.txt", cloud);

    std::cout << "> done testing the normal estimation function." << std::endl << std::endl;
}

void test_eff_norm_est()
{
    // point cloud to be examined
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

    std::cout << "> efficient normal estimation function test start..." << std::endl;

    std::cout << "> loading cloud from file..." << std::endl;
    cloud = pcloud_io::load_cloud("../../../data/widop_test.txt", false);
    std::cout << "> finished loading cloud." << std::endl;

    clock_t t_start = clock();
    std::cout << "> launching eff_norm_est function..." << std::endl;
    colored_cloud = eff_norm_est(cloud, RADIUS_WIDOP, 50, 100);
    std::cout << "> eff_norm_est function finished executing. execution time: "
              << (float)(clock() - t_start)/CLOCKS_PER_SEC << " seconds." << std::endl;

    std::cout << "> writing results to text file..." << std::endl;
    pcloud_io::cloud_to_txt("../../../data/test_results/widop_testeff.txt", colored_cloud);
    std::cout << "> done writing." << std::endl;

    std::cout << "> efficient normal estimation function test ended." << std::endl << std::endl;
}

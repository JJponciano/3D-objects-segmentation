#include "test_lib.h"

void test_load_cloud()
{
    try
    {
        pcloud_io::load_cloud("../../../data/bad_file.txt", false);
    }

    catch(char const* io_err)
    {
        std::cout << io_err << std::endl;
    }
}

void test_normal_estimation()
{
    // input
    float radius;
    int max_neighbs;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);   // cloud to be examined

    std::cout << "> loading cloud from file..." << std::endl;
    cloud = pcloud_io::load_cloud("../../../data/table.txt", false);
    std::cout << std::endl << "> finished loading cloud." << std::endl;

    std::cout << "> normal estimation function test start..." << std::endl;

    std::cout << "> radius: ";
    std::cin >> radius;
    std::cout << std::endl << "> max_neighbs: ";
    std::cin >> max_neighbs;
    std::cout << std::endl;

    std::cout << "> estimating normals..." << std::endl;
    clock_t t_start = clock();

    try
    {
        normal_estimation(cloud, radius, max_neighbs);
        std::cout << "> done estimating normals; execution time: " << (float)(clock() - t_start)/CLOCKS_PER_SEC << " seconds." << std::endl;

        std::cout << "> writing cloud to file..." << std::endl;
        pcloud_io::write_cloud("../../../data/test_results/norm_est/normal_estimation_test_"
                               + boost::lexical_cast<std::string>(radius) + "_"
                               + boost::lexical_cast<std::string>(max_neighbs)
                               + "_" + ".txt", cloud);
        std::cout << "> done writing." << std::endl;

        std::cout << "> done testing the normal estimation function." << std::endl << std::endl;
    }

    catch (std::logic_error const& err)
    {
        std::cout << err.what() << std::endl;
    }
}

void test_e_normal_estimation()
{
    // input
    float radius;
    int max_neighbs;
    int y_scale;

    // clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;   // cloud to be examined
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;   // output cloud

    std::cout << "> loading cloud from file..." << std::endl;
    cloud = pcloud_io::load_cloud("../../../data/widop_test.txt", false);
    std::cout << "> finished loading cloud." << std::endl;

    std::cout << "> efficient normal estimation function test start..." << std::endl;

    std::cout << "> radius: ";
    std::cin >> radius;
    std::cout << std::endl << "> max_neighbs: ";
    std::cin >> max_neighbs;
    std::cout << std::endl << "> y_scale: ";
    std::cin >> y_scale;
    std::cout << std::endl;

    std::cout << "> launching e_normal_estimation function function..." << std::endl;
    clock_t t_start = clock();
    try
    {
        colored_cloud = e_normal_estimation(cloud, radius, max_neighbs, 1.0, y_scale, 1.0);
        std::cout << "> e_normal_estimation function finished executing. execution time: "
                  << (float)(clock() - t_start)/CLOCKS_PER_SEC << " seconds." << std::endl;

        std::cout << "> writing results to text file..." << std::endl;
        pcloud_io::write_cloud("../../../data/test_results/eff_norm_est/e_normal_estimation_test"
                                + boost::lexical_cast<std::string>(radius) + "_"
                                + boost::lexical_cast<std::string>(max_neighbs) + "_"
                                + boost::lexical_cast<std::string>(y_scale)
                                + ".txt", colored_cloud);
        std::cout << "> done writing." << std::endl;

        std::cout << "> efficient normal estimation function test ended." << std::endl << std::endl;
    }

    catch(std::exception const& err)
    {
        std::cout << err.what() << std::endl;
    }
}

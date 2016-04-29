#include "test_lib.h"

void test_import_cloud(std::string bad_path)
{
    try
    {
        pcloud_io::import_cloud(bad_path, false);
    }

    catch(char const* io_err)
    {
        std::cout << io_err << std::endl;
    }
}

void test_normal_estimation(std::string import_path,
                              std::string export_path,
                              float radius,
                              int max_neighbs)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);   // cloud to be examined

    try
    {
        cloud = pcloud_io::import_cloud(import_path, false);
        normal_estimation(cloud, radius, max_neighbs);
        pcloud_io::export_cloud(export_path
                               + boost::lexical_cast<std::string>(radius) + "_"
                               + boost::lexical_cast<std::string>(max_neighbs)
                               + "_" + ".txt", cloud);
    }

    catch (std::exception const& err)
    {
        std::string err_string = "test_normal_estimation : ";
        err_string.append(err.what());
        throw err_string;
    }
}

void test_e_normal_estimation(std::string import_path,
                              std::string export_path,
                              float radius,
                              int max_neighbs,
                              std::vector<float> xyzscale)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;   // cloud to be examined
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;   // output cloud

    try
    {
        cloud = pcloud_io::import_cloud(import_path, false);
        colored_cloud = e_normal_estimation(cloud, radius, max_neighbs, xyzscale[0], xyzscale[1], xyzscale[2]);
        pcloud_io::export_cloud(export_path + boost::lexical_cast<std::string>(radius) + "_"
                                + boost::lexical_cast<std::string>(max_neighbs) + "_"
                                + boost::lexical_cast<std::string>(xyzscale[0]) + "_"
                                + boost::lexical_cast<std::string>(xyzscale[1]) + "_"
                                + boost::lexical_cast<std::string>(xyzscale[2]) + "_"
                                + ".txt", colored_cloud);
    }

    catch(std::exception const& err)
    {
        std::string err_string = "test_normal_estimation : ";
        err_string.append(err.what());
        throw err_string;
    }
}


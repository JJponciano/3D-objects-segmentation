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
                              int is_rgb,
                              float radius,
                              int max_neighbs)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);   // cloud to be examined

    try
    {
        cloud = pcloud_io::import_cloud(import_path, is_rgb);
        normal_estimation(cloud, radius, max_neighbs);
        pcloud_io::export_cloud(export_path + "normal_estimation_test_"
                               + boost::lexical_cast<std::string>(radius)
                               + "_" + boost::lexical_cast<std::string>(max_neighbs)
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
                              int is_rgb,
                              float radius,
                              int max_neighbs,
                              std::vector<float> xyzscale,
                              float precision)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;   // output cloud

    try
    {
        cloud = pcloud_io::import_cloud(import_path, is_rgb);
        colored_cloud = e_normal_estimation(cloud, radius, max_neighbs,
                                            xyzscale[0], xyzscale[1], xyzscale[2],
                                            precision);
        pcloud_io::export_cloud(export_path + "e_normal_estimation_test_"
                                + boost::lexical_cast<std::string>(radius) + "_"
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

void test_cloud_homogenization(std::string import_path,
                               std::string export_path,
                               int is_rgb,
                               short epsilon)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcloud_io::import_cloud(import_path, is_rgb);

    cloud_manip::cloud_homogenization(cloud, epsilon);

    pcloud_io::export_cloud(export_path + "cloud_homogenization_test_"
                            + boost::lexical_cast<std::string>(epsilon)
                            + ".txt", cloud);
}

void test_crop_cloud(std::string import_path,
                     std::string export_path,
                     int is_rgb,
                     std::vector<float> xyzthresh,
                     float precision)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud;

    try
    {
        cloud = pcloud_io::import_cloud(import_path, is_rgb);
        cropped_cloud =
                cloud_manip::crop_cloud(cloud, xyzthresh[0], xyzthresh[1], xyzthresh[2], precision);
        pcloud_io::export_cloud(export_path + "cloud_crop_test_"
                                + boost::lexical_cast<std::string>(xyzthresh[0]) + "_"
                                + boost::lexical_cast<std::string>(xyzthresh[1]) + "_"
                                + boost::lexical_cast<std::string>(xyzthresh[2]) + "_"
                                + ".txt", cropped_cloud);
    }

    catch(std::exception const& err)
    {
        std::string err_string = "test_crop_cloud : ";
        err_string.append(err.what());
        throw err_string;
    }
}

float test_precision(float float_num, float precision) { return geom::aux::set_precision(float_num, precision); }

void test_color_to_greyscale(std::string import_path,
                             std::string export_path,
                             int is_rgb)

{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    std::vector<point_xy_greyscale> greyscale_result;

    try
    {
        cloud = pcloud_io::import_cloud(import_path, is_rgb);
        greyscale_result = cloud_manip::cloud_to_greyscale(cloud);
        pcloud_io::export_greyscale(export_path + "color_to_greyscale_test"
                                    + "_.txt", greyscale_result);
    }

    catch(std::exception const& err)
    {
        std::string err_string = "test_color_to_greyscale: ";
        err_string.append(err.what());
        throw err_string;
    }

    catch(char const* char_ptr_err)
    {
        throw char_ptr_err;
    }

    catch(std::string str_err)
    {
        throw str_err;
    }
}

void test_greyscale_to_image(std::string import_path, std::string export_path,
                             std::string magic_number,
                             int is_rgb,
                             float epsilon)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    std::vector<point_xy_greyscale> greyscale_result;

    try
    {
        cloud = pcloud_io::import_cloud(import_path, is_rgb);
        greyscale_result = cloud_manip::cloud_to_greyscale(cloud);
        greyscale_image gs_img= cloud_manip::greyscale_to_image(greyscale_result, epsilon);
        pcloud_io::export_image(export_path + "greyscale_to_image_test_"
                                + boost::lexical_cast<std::string>(epsilon)
                                + "_.pgm", magic_number, gs_img);
    }

    catch(std::exception const& err)
    {
        std::string err_string = "test_greyscale_to_image: ";
        err_string.append(err.what());
        throw err_string;
    }

    catch(char const* char_ptr_err)
    {
        throw char_ptr_err;
    }

    catch(std::string str_err)
    {
        throw str_err;
    }
}

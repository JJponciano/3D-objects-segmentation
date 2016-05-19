#include "test_lib.h"

void test_import_cloud(std::string bad_path)
{
    try
    {
        cloud_io::import_cloud(bad_path, false);
    }

    catch(char const* io_err)
    {
        std::cout << io_err << std::endl;
    }
}

void test_normal_estimation(std::string import_path, std::string export_path, int is_rgb,
                              float radius, int max_neighbs)
{
    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);   // cloud to be examined

        cloud = cloud_io::import_cloud(import_path, is_rgb);
        normal_estimation(cloud, radius, max_neighbs);
        cloud_io::export_cloud(export_path + "normal_estimation_test_"
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

void test_e_normal_estimation(std::string import_path, std::string export_path, int is_rgb,
                              float radius, int max_neighbs, std::vector<float> xyzscale,
                              float max_fragment_depth, float precision)
{
    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;   // output cloud

        cloud = cloud_io::import_cloud(import_path, is_rgb);
        colored_cloud = e_normal_estimation(cloud, max_neighbs, radius,
                                            xyzscale[0], xyzscale[1], xyzscale[2],
                                            max_fragment_depth, precision);
        cloud_io::export_cloud(export_path + "e_normal_estimation_test_"
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

void test_cloud_homogenization(std::string import_path, std::string export_path,
                               int is_rgb, short epsilon)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_io::import_cloud(import_path, is_rgb);

    cloud_manip::cloud_homogenization(cloud, epsilon);

    cloud_io::export_cloud(export_path + "cloud_homogenization_test_"
                            + boost::lexical_cast<std::string>(epsilon)
                            + ".txt", cloud);
}

void test_crop_cloud(std::string import_path,
                     std::string export_path,
                     int is_rgb,
                     std::vector<float> xyzthresh,
                     float precision)
{
    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud;

        cloud = cloud_io::import_cloud(import_path, is_rgb);
        cropped_cloud =
                cloud_manip::crop_cloud(cloud, xyzthresh[0], xyzthresh[1], xyzthresh[2], precision);
        cloud_io::export_cloud(export_path + "cloud_crop_test_"
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

float test_precision(float float_num, float precision) { return aux::set_precision(float_num, precision); }

void test_color_to_greyscale(std::string import_path,
                             std::string export_path,
                             int is_rgb)

{
    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        std::vector<point_xy_greyscale> greyscale_result;

        cloud = cloud_io::import_cloud(import_path, is_rgb);
        greyscale_result = cloud_manip::cloud_to_2d_greyscale(cloud);
        image_io::export_greyscale_vector(export_path + "color_to_greyscale_test"
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

void test_greyscale_to_image(std::string import_path, std::string export_path, int is_rgb, float epsilon)
{
    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        std::vector<point_xy_greyscale> greyscale_result;

        cloud = cloud_io::import_cloud(import_path, is_rgb);
        greyscale_result = cloud_manip::cloud_to_2d_greyscale(cloud);
        image_greyscale gs_img= image_processing::greyscale_vector_to_image(greyscale_result, epsilon);
        image_processing::normalize(&gs_img);
        image_io::export_greyscale_image(export_path + "greyscale_to_image_test_"
                                + boost::lexical_cast<std::string>(epsilon)
                                + "_.pgm", 255, gs_img);
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

void test_image_to_cloud(std::string import_path, std::string export_path, int is_rgb, float epsilon)
{
    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr reverse_cloud;
        std::vector<point_xy_mixed> mixed_result;

        cloud = cloud_io::import_cloud(import_path, is_rgb);
        mixed_result = cloud_manip::cloud_to_2d_mixed(cloud);

        image_mixed mixed_img = image_processing::mixed_vector_to_image(mixed_result, epsilon);
        reverse_cloud = image_processing::mixed_image_to_cloud(mixed_img, cloud);
        cloud_io::export_cloud(export_path + "cloud_remade_"
                                + boost::lexical_cast<std::string>(epsilon)
                                + "_.txt", reverse_cloud);
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

void test_rail_detection(std::string import_path, std::string export_path, int is_rgb, float epsilon)
{
    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr railway_cloud;
        std::vector<point_xy_mixed> mixed_result;

        base_cloud = cloud_io::import_cloud(import_path, is_rgb);
        cloud_manip::cloud_homogenization(base_cloud, 200);
        mixed_result = cloud_manip::cloud_to_2d_mixed(base_cloud);

        image_mixed mixed_img = image_processing::mixed_vector_to_image(mixed_result, epsilon);

        for (unsigned long y = 0; y < mixed_img.height(); y++)
        {
            for (unsigned long x = 0; x < mixed_img.width(); x++)
            {
                if (mixed_img.get_grey_at(y, x) == 0)
                    mixed_img.set_rgb_at(y, x, (uint32_t)0);

                if (mixed_img.get_blue_at(y, x) == (uint8_t)200)
                    mixed_img.set_rgb_at(y, x, (uint32_t)0);
            }
        }

        railway_cloud = image_processing::mixed_image_to_cloud(mixed_img, base_cloud);
        cloud_io::export_cloud(export_path + "railway_detection_test_"
                                + boost::lexical_cast<std::string>(epsilon)
                                + "_.txt", railway_cloud);
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

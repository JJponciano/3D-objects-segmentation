#include "test_lib.h"

void test_normal_estimation(std::string import_path, std::string export_path, int is_rgb,
                            float radius, int max_neighbs, float x_scale, float y_scale, float z_scale,
                            float max_fragment_depth, float precision)
{
    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;   // output cloud

        base_cloud = cloud_io::import_cloud(import_path, is_rgb);
        colored_cloud = fast_normal_estimation(base_cloud, max_neighbs, radius, x_scale, y_scale, z_scale, max_fragment_depth, precision);
        cloud_io::export_cloud(export_path + "multithreaded_normal_estimation_test_" + boost::lexical_cast<std::string>(radius) + "_"
                                + boost::lexical_cast<std::string>(max_neighbs) + "_" + boost::lexical_cast<std::string>(x_scale) + "_"
                                + boost::lexical_cast<std::string>(y_scale) + "_" + boost::lexical_cast<std::string>(z_scale)
                                + ".txt", colored_cloud);
    }

    catch(std::exception err)
    {
        QString err_msg;

        err_msg.append(err.what());
        throw err_msg;
    }

    catch(QString err_msg)
    {
        throw err_msg;
    }
}

void test_cloud_homogenization(std::string import_path, std::string export_path, int is_rgb, short epsilon)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud = cloud_io::import_cloud(import_path, is_rgb);

    cloud_manip::cloud_homogenization(base_cloud, epsilon);
    cloud_io::export_cloud(export_path + "cloud_homogenization_test_" + boost::lexical_cast<std::string>(epsilon)
                            + ".txt", base_cloud);
}

void test_crop_cloud(std::string import_path, std::string export_path, int is_rgb,
                     float x_thresh, float y_thresh, float z_thresh, float precision)
{
    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud;

        base_cloud = cloud_io::import_cloud(import_path, is_rgb);
        cropped_cloud = cloud_manip::crop_cloud(base_cloud, x_thresh, y_thresh, z_thresh, precision);
        cloud_io::export_cloud(export_path + "cloud_crop_test_" + boost::lexical_cast<std::string>(x_thresh) + "_"
                                + boost::lexical_cast<std::string>(y_thresh) + "_" + boost::lexical_cast<std::string>(z_thresh)
                                + ".txt", cropped_cloud);
    }

    catch(std::exception err)
    {
        QString err_msg;

        err_msg.append(err.what());
        throw err_msg;
    }

    catch(QString err_msg)
    {
        throw err_msg;
    }
}

void test_greyscale_image_to_file(std::string import_path, std::string export_path, int is_rgb, float epsilon)
{
    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        std::vector<point_xy_mixed> mixed_result;

        cloud = cloud_io::import_cloud(import_path, is_rgb);
        mixed_result = cloud_manip::cloud_to_2d_mixed(cloud);

        image_mixed mixed_img = image_processing::mixed_vector_to_image(mixed_result, epsilon);
        image_greyscale gs_img = image_processing::mixed_image_to_greyscale(mixed_img);
        image_io::export_greyscale_image(export_path + "gscale_to_image_test_" + boost::lexical_cast<std::string>(epsilon)
                                + "_.pgm", 255, gs_img);
    }

    catch(std::exception err)
    {
        std::string err_string = "test_greyscale_to_image: ";
        err_string.append(err.what());
        throw err_string;
    }

    catch(QString err_msg)
    {
        throw err_msg;
    }
}

void test_rgb_image_to_file(std::string import_path, std::string export_path, int is_rgb, float epsilon)
{
    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        std::vector<point_xy_mixed> mixed_result;

        cloud = cloud_io::import_cloud(import_path, is_rgb);
        mixed_result = cloud_manip::cloud_to_2d_mixed(cloud);

        image_mixed mixed_img = image_processing::mixed_vector_to_image(mixed_result, epsilon);
        image_rgb rgb_img = image_processing::mixed_image_to_rgb(mixed_img);
        image_io::export_rgb_image(export_path + "rgb_to_image_test_" + boost::lexical_cast<std::string>(epsilon)
                                + "_.ppm", 255, rgb_img);
    }

    catch(std::exception const& err)
    {
        std::string err_string = "test_rgb_to_image: ";
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

void test_mixed_image_to_cloud(std::string import_path, std::string export_path, int is_rgb, float epsilon)
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
        std::string err_string = "test_image_to_cloud: ";
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
        std::string err_string = "test_raiL_detection: ";
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

#include "test_lib.h"

int test_normal_estimation(std::string import_path, std::string export_path, float radius,
                            int max_neighbs, float x_scale, float y_scale, float z_scale,
                            float max_fragment_depth)
{
    int code = 0;

    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud_ptr;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_ptr;   // output cloud
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_fragments;
        float max_scaled_fragment_depth = max_fragment_depth / y_scale;

        base_cloud_ptr = cloud_io::import_cloud(import_path);
        cloud_manip::scale_cloud(base_cloud_ptr, x_scale, y_scale, z_scale); // scaling cloud
        cloud_fragments = cloud_manip::fragment_cloud(base_cloud_ptr, max_scaled_fragment_depth); // fragmenting cloud for less execution time

        // estimating the normals for each cloud fragment in parallel
        #pragma omp parallel for schedule(static)
        for (auto fragm_it = cloud_fragments.begin(); fragm_it < cloud_fragments.end(); fragm_it++)
            normal_segmentation::estimate_normals(*fragm_it, radius, max_neighbs);

        colored_cloud_ptr = cloud_manip::merge_clouds(cloud_fragments); // merging fragments to build original cloud
        cloud_manip::scale_cloud(colored_cloud_ptr, (1.0/x_scale), (1.0/y_scale), (1.0/z_scale));    // restoring widop scale
        cloud_io::export_cloud(export_path + "/normal_estimation_test_" + boost::lexical_cast<std::string>(radius) + "_"
                                + boost::lexical_cast<std::string>(max_neighbs) + "_" + boost::lexical_cast<std::string>(x_scale) + "_"
                                + boost::lexical_cast<std::string>(y_scale) + "_" + boost::lexical_cast<std::string>(z_scale) + "_"
                                + boost::lexical_cast<std::string>(max_fragment_depth) + ".txt", colored_cloud_ptr);

    }

    catch (...)
    {
        code = -1;
    }

    return code;
}

int test_cloud_homogenization(std::string import_path, std::string export_path, short epsilon)
{
    int code = 0;

    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = cloud_io::import_cloud(import_path);

        cloud_manip::homogenize_cloud(cloud_ptr, epsilon);
        cloud_io::export_cloud(export_path + "/cloud_homogenization_test_" + boost::lexical_cast<std::string>(epsilon)
                                + ".txt", cloud_ptr);
    }

    catch (...)
    {
        code = -1;
    }

    return code;
}

int test_crop_cloud(std::string import_path, std::string export_path,
                     float x_thresh, float y_thresh, float z_thresh)
{
    int code = 0;

    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;

        cloud_ptr = cloud_io::import_cloud(import_path);
        cloud_manip::crop_cloud(cloud_ptr, x_thresh, y_thresh, z_thresh);
        cloud_io::export_cloud(export_path + "/cloud_crop_test_" + boost::lexical_cast<std::string>(x_thresh) + "_"
                                + boost::lexical_cast<std::string>(y_thresh) + "_" + boost::lexical_cast<std::string>(z_thresh)
                                + ".txt", cloud_ptr);
    }

    catch (...)
    {
        code = -1;
    }

    return code;
}

int test_greyscale_image_to_file(std::string import_path, std::string export_path, float epsilon)
{
    int code = 0;

    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        std::vector<point_xy_mixed> mixed_result;

        cloud = cloud_io::import_cloud(import_path);
        mixed_result = cloud_manip::cloud_to_2d_mixed(cloud);

        image_mixed mixed_img = image_processing::mixed_vector_to_image(mixed_result, epsilon);
        image_greyscale gs_img = image_processing::mixed_image_to_greyscale(mixed_img);
        image_io::export_greyscale_image(export_path + "/gscale_to_image_test_" + boost::lexical_cast<std::string>(epsilon)
                                + ".pgm", 255, gs_img);
    }

    catch (...)
    {
        code = -1;
    }

    return code;
}

int test_rgb_image_to_file(std::string import_path, std::string export_path, float epsilon)
{
    int code = 0;

    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        std::vector<point_xy_mixed> mixed_result;

        cloud = cloud_io::import_cloud(import_path);
        mixed_result = cloud_manip::cloud_to_2d_mixed(cloud);

        image_mixed mixed_img = image_processing::mixed_vector_to_image(mixed_result, epsilon);
        image_rgb rgb_img = image_processing::mixed_image_to_rgb(mixed_img);
        image_io::export_rgb_image(export_path + "/rgb_to_image_test_" + boost::lexical_cast<std::string>(epsilon)
                                + ".ppm", 255, rgb_img);
    }

    catch (...)
    {
        code = -1;
    }

    return code;
}

int test_mixed_image_to_cloud(std::string import_path, std::string export_path, float epsilon)
{
    int code = 0;

    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr reverse_cloud;
        std::vector<point_xy_mixed> mixed_result;

        cloud = cloud_io::import_cloud(import_path);
        mixed_result = cloud_manip::cloud_to_2d_mixed(cloud);

        image_mixed mixed_img = image_processing::mixed_vector_to_image(mixed_result, epsilon);
        reverse_cloud = image_processing::mixed_image_to_cloud(mixed_img, cloud);
        cloud_io::export_cloud(export_path + "cloud_remade_"
                                + boost::lexical_cast<std::string>(epsilon)
                                + "_.txt", reverse_cloud);
    }

    catch (...)
    {
        code = -1;
    }

    return code;
}

int test_rail_detection(std::string import_path, std::string export_path, float epsilon)
{
    int code = 0;

    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr railway_cloud;
        std::vector<point_xy_mixed> mixed_result;

        base_cloud = cloud_io::import_cloud(import_path);
        cloud_manip::homogenize_cloud(base_cloud, 200);
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

    catch (...)
    {
        code = -1;
    }

    return code;
}

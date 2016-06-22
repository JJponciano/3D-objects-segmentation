#include "test_lib.h"

using namespace cloud_object_segmentation;

int test::crop_cloud(std::string cloud_import_path, std::string cloud_export_path,
                     float x_thresh, float y_thresh, float z_thresh)
{
    int code = 0;

    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud_ptr;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud_ptr;

        base_cloud_ptr = io::import_cloud(cloud_import_path);
        cropped_cloud_ptr = cloud_manip::crop_cloud(base_cloud_ptr, x_thresh, y_thresh, z_thresh);
        io::export_cloud(cloud_export_path + "/cloud_crop" + boost::lexical_cast<std::string>(x_thresh) + "_"
                                + boost::lexical_cast<std::string>(y_thresh) + "_" + boost::lexical_cast<std::string>(z_thresh)
                                + ".txt", cropped_cloud_ptr);
    }

    catch (...)
    {
        code = -1;
    }

    return code;
}

int test::estimate_normals(std::string cloud_import_path, std::string cloud_export_path, float radius,
                            int max_neighbs, float x_scale, float y_scale, float z_scale,
                            float max_fragment_depth)
{
    int code = 0;

    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud_ptr;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_ptr;   // output cloud
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_fragments;
        float max_scaled_fragment_depth = max_fragment_depth * y_scale;

        base_cloud_ptr = io::import_cloud(cloud_import_path);
        cloud_manip::scale_cloud(base_cloud_ptr, x_scale, y_scale, z_scale); // scaling cloud
        cloud_fragments = cloud_manip::fragment_cloud(base_cloud_ptr, max_scaled_fragment_depth); // fragmenting cloud for less execution time

        // estimating the normals for each cloud fragment in parallel
        #pragma omp parallel for schedule(static)
        for (unsigned long fragm_it = 0; fragm_it < cloud_fragments.size(); fragm_it++)
           normal_segmentation::estimate_normals(cloud_fragments[fragm_it], radius, max_neighbs);

        colored_cloud_ptr = cloud_manip::merge_clouds(cloud_fragments); // merging fragments to build original cloud
        cloud_manip::scale_cloud(colored_cloud_ptr, (1.0/x_scale), (1.0/y_scale), (1.0/z_scale));    // restoring widop scale
        io::export_cloud(cloud_export_path + "/normal_estimation" + boost::lexical_cast<std::string>(radius) + "_"
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

int test::homogenize_cloud(std::string cloud_import_path, std::string cloud_export_path, short epsilon)
{
    int code = 0;

    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = io::import_cloud(cloud_import_path);

        cloud_manip::homogenize_cloud(cloud_ptr, epsilon);
        io::export_cloud(cloud_export_path + "/cloud_homogenization" + boost::lexical_cast<std::string>(epsilon)
                                + ".txt", cloud_ptr);
    }

    catch (...)
    {
        code = -1;
    }

    return code;
}

int test::cloud_to_image(int img_type, std::string cloud_import_path, std::string img_export_path,
                         size_t width, size_t height)
{
    int code = 0;

    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = io::import_cloud(cloud_import_path);
        std::vector<point_xy_mixed> mixed_vector = cloud_manip::cloud_to_2d_mixed(cloud_ptr);
        image_mixed mixed_img = image_processing::mixed_vector_to_image(mixed_vector, width, height);

        if (img_type == 0)
        {
            image_greyscale gs_img = image_processing::cloud_to_depth_image(cloud_ptr, width, height);
            io::export_greyscale_image(img_export_path + "/cloud_to_greyscale_image.pgm", 255, gs_img);
        }

        else if (img_type == 1)
        {
            image_rgb rgb_img = image_processing::mixed_image_to_rgb(mixed_img);
            io::export_rgb_image(img_export_path + "/cloud_to_rgb_img.ppm", 255, rgb_img);
        }

        else throw std::runtime_error("Index out of bounds.");
    }

    catch (...)
    {
        code = -1;
    }

    return code;
}

int test::detect_contours(std::string img_import_path, std::string img_export_path)
{
    int code = 0;

    try
    {
        image_greyscale gs_img = io::import_greyscale_image(img_import_path);
        image_greyscale res_img = image_processing::detect_contours(gs_img, 255);

        io::export_greyscale_image(img_export_path + "/contour_detection_test.pgm", 255, res_img);
    }

    catch (...)
    {
        code = -1;
    }

    return code;
}

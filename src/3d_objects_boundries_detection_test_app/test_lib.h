/**
  * @brief test library
  */

#ifndef TEST_LIB_H
#define TEST_LIB_H

#include "../algorithms/normal_segmentation/normal_estimation.h"
#include "../algorithms/tools/image_processing.h"
#include "../algorithms/tools/cloud_manip.h"
#include "../algorithms/io/cloud_io.h"
#include "../algorithms/io/image_io.h"

namespace test
{
    /// structured test input
    struct io_data
    {
        std::string file_in_path;
        std::string file_out_path;
    };

    struct epsilon_data : io_data
    {
        float epsilon;
    };

    struct thresh_data : io_data
    {
        float x_thresh;
        float y_thresh;
        float z_thresh;
    };

    struct normal_estimation_data : io_data
    {
        float radius;
        float max_neighbs;
        float x_scale;
        float y_scale;
        float z_scale;
        float max_fragment_depth;
    };

    /// test functions
    int crop_cloud(std::string cloud_import_path, std::string cloud_export_path,
                    float x_thresh, float y_thresh, float z_thresh);

    int estimate_normals(std::string cloud_import_path, std::string cloud_export_path, float radius, int max_neighbs,
                                  float x_scale, float y_scale, float z_scale, float max_fragment_depth);

    int homogenize_cloud(std::string cloud_import_path, std::string cloud_export_path, short color_epsilon);

    int cloud_to_image(int img_type, std::string cloud_import_path, std::string img_export_path, float x_epsilon);
}

#endif // TEST_LIB_H

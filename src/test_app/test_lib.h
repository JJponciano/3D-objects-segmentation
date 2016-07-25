/**
  * @brief test library
  */

#ifndef TEST_LIB_H
#define TEST_LIB_H

#include "../cos_lib/include/normal_estimation.h"
#include "../cos_lib/include/image_processing.h"
#include "../cos_lib/include/cloud_manip.h"
#include "../cos_lib/include/cloud_io.h"
#include "../cos_lib/include/image_io.h"

namespace test
{
    /// test functions
    int crop_cloud(std::string cloud_import_path, std::string cloud_export_path,
                    float x_thresh, float y_thresh, float z_thresh);

    int estimate_normals(std::string cloud_import_path, std::string cloud_export_path, float radius, int max_neighbs,
                                  float x_scale, float y_scale, float z_scale, float max_fragment_depth);

    int homogenize_cloud(std::string cloud_import_path, std::string cloud_export_path, short color_epsilon);

    int cloud_to_image(int img_type, std::string cloud_import_path, std::string img_export_path,
                       size_t width, size_t height);

    int detect_contours(std::string img_import_path, std::string img_export_path);
}

#endif // TEST_LIB_H

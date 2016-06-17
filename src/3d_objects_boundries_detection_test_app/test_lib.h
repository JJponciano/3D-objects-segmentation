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
    /// test functions
    int crop_cloud(std::string cloud_import_path, std::string cloud_export_path,
                    float x_thresh, float y_thresh, float z_thresh);

    int estimate_normals(std::string cloud_import_path, std::string cloud_export_path, float radius, int max_neighbs,
                                  float x_scale, float y_scale, float z_scale, float max_fragment_depth);

    int homogenize_cloud(std::string cloud_import_path, std::string cloud_export_path, short color_epsilon);

    int cloud_to_image(int img_type, std::string cloud_import_path, std::string img_export_path, float x_epsilon);

    int detect_contours(std::string img_import_path, std::string img_export_path);
}

#endif // TEST_LIB_H

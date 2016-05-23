/**
  * @brief test library
  */

#ifndef TEST_LIB_H
#define TEST_LIB_H

#include "../normal_segmentation/fast_normal_estimation.h"
#include "../tools/image_processing.h"
#include "../tools/cloud_manip.h"
#include "../io/cloud_io.h"
#include "../io/image_io.h"

#include <time.h>

// test functions
void test_normal_estimation(std::string import_path, std::string export_path, int is_rgb, float radius, int max_neighbs,
                              float x_scale, float y_scale, float z_scale, float max_fragment_depth, float precision);

void test_cloud_homogenization(std::string import_path, std::string export_path,
                               int is_rgb, short epsilon);

void test_crop_cloud(std::string import_path, std::string export_path, int is_rgb,
                     std::vector<float> xyzthresh, float precision);

void test_greyscale_image_to_file(std::string import_path, std::string export_path, int is_rgb, float epsilon);

void test_rgb_image_to_file(std::string import_path, std::string export_path, int is_rgb, float epsilon);

void test_mixed_image_to_cloud(std::string import_path, std::string export_path, int is_rgb, float epsilon);

void test_rail_detection(std::string import_path, std::string export_path, int is_rgb, float epsilon);

#endif // TEST_LIB_H

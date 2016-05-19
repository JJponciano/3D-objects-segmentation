/**
  * @brief test library
  */

#ifndef TEST_LIB_H
#define TEST_LIB_H

#include "e_normal_estimation.h"
#include "../tools/image_processing.h"
#include "../tools/cloud_manip.h"
#include "../io/cloud_io.h"
#include "../io/image_io.h"

#include <time.h>

// test functions
void test_load_cloud(std::string bad_path);

void test_normal_estimation(std::string import_path, std::string export_path,
                            int is_rgb, float radius, int max_neighbs);

void test_e_normal_estimation(std::string import_path, std::string export_path,
                              int is_rgb, float radius, int max_neighbs,
                              std::vector<float> xyzscale, float max_fragment_depth,
                              float precision);

void test_cloud_homogenization(std::string import_path, std::string export_path,
                               int is_rgb, short epsilon);

void test_crop_cloud(std::string import_path, std::string export_path, int is_rgb,
                     std::vector<float> xyzthresh, float precision);


float test_precision(float float_num, float precision);

void test_color_to_greyscale(std::string import_path, std::string export_path, int is_rgb);

void test_greyscale_to_image(std::string import_path, std::string export_path, int is_rgb, float epsilon);

void test_image_to_cloud(std::string import_path, std::string export_path, int is_rgb, float epsilon);

void test_rail_detection(std::string import_path, std::string export_path, int is_rgb, float epsilon);

#endif // TEST_LIB_H

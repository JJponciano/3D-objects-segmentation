/**
  * @brief test library
  */

#ifndef TEST_LIB_H
#define TEST_LIB_H

#include "e_normal_estimation.h"
#include "cloud_manip.h"
#include "../io/pcloud_io.h"

#include <time.h>

// test functions
void test_load_cloud(std::string bad_path);

void test_normal_estimation(std::string import_path,
                            std::string export_path,
                            float radius,
                            int max_neighbs);

void test_e_normal_estimation(std::string import_path,
                              std::string export_path,
                              float radius,
                              int max_neighbs,
                              std::vector<float> xyzscale);



#endif // TEST_LIB_H

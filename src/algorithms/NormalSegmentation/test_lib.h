/**
  * @brief test library
  */

#ifndef TEST_LIB_H
#define TEST_LIB_H

#include "normal_estimation.h"
#include "e_normal_estimation.h"
#include "cloud_manip.h"
#include "../io/pcloud_io.h"

#include <time.h>

// test functions
void test_load_cloud();
void test_normal_estimation();
void test_e_normal_estimation();

#endif // TEST_LIB_H

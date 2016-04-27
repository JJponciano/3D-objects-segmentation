/**
  * @brief test library
  */

#ifndef TEST_LIB_H
#define TEST_LIB_H

#include "normal_estimation.h"
#include "eff_norm_est.h"
#include "cloud_manip.h"
#include "../io/pcloud_io.h"

#include <time.h>

#define RADIUS 0.01 /** recommended for the table test **/
#define RADIUS_WIDOP 0.5   /** recommended for the widop test **/

#define MAX_NEIGHBS 100

#define Y_SCALE 1000

// test functions
void test_normal_estimation();
void test_eff_norm_est();

#endif // TEST_LIB_H

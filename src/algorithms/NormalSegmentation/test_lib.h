/**
  * @brief test library
  */

#ifndef TEST_LIB_H
#define TEST_LIB_H

#include "normal_estimation.h"
<<<<<<< HEAD
#include "../io/pcloud_io.h"

#define RADIUS 0.01

void test_normal_estimation();
=======
#include "cloud_manip.h"
#include "../io/pcloud_io.h"

#define RADIUS 0.01 /** recommended for the table test **/
#define RADIUS_WIDOP 0.05   /** recommended for the widop test **/

void test_normal_estimation();
void test_scaling();
>>>>>>> NormalSegmentation

#endif // TEST_LIB_H

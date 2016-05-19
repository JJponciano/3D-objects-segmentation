#ifndef INTERFACE_H
#define INTERFACE_H

#include "../test_lib/test_lib.h"

#define NORMAL_ESTIMATION_RES_OUTPUT_PATH "/home/vlad-adrian/share/sem_detection/semanticDir/test/normal_estimation_test_results/result_clouds/"
#define E_NORMAL_ESTIMATION_RES_OUTPUT_PATH "/home/vlad-adrian/share/sem_detection/semanticDir/test/e_normal_estimation_test_results/result_clouds/"
#define CONVERT_TO_GREYSCALE_RES_OUTPUT_PATH "/home/vlad-adrian/share/sem_detection/semanticDir/test/xyzrgb_to_xygreyscale_test_results/result_clouds/"
#define CLOUD_CROP_RES_OUTPUT_PATH "/home/vlad-adrian/share/sem_detection/semanticDir/test/cloud_crop_test_results/result_clouds/"
#define CLOUD_HOMOG_RES_OUTPUT_PATH "/home/vlad-adrian/share/sem_detection/semanticDir/test/cloud_homog_test_results/result_clouds/"
#define GREYSCALE_TO_IMAGE_RES_OUTPUT_PATH "/home/vlad-adrian/share/sem_detection/semanticDir/test/greyscale_to_image_test_results/result_clouds/"
#define IMAGE_TO_FILE_RES_OUTPUT_PATH "/home/vlad-adrian/share/sem_detection/semanticDir/test/image_to_file/text_results/"
#define IMAGE_TO_CLOUD_RES_OUTPUT_PATH "/home/vlad-adrian/share/sem_detection/semanticDir/test/image_to_cloud/cloud_results/"

float get_precision_input();
float get_radius_input();
float get_z_min_input();
float get_z_max_input();
float get_float_input();
float get_dec_precision_input();
float get_epsilon_input();
float get_max_fragment_depth_input();
int get_max_neighbs_input();
int get_file_type_input();
std::vector<float> get_xyz_input();
std::string get_import_path_input();
std::string get_export_path_input();
std::string get_magic_number_input();
void clear_screen();
void invalid_input();
void success();
void failure(char const* err);
void test_menu();

#endif // INTERFACE_H

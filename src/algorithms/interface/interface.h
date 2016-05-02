#ifndef INTERFACE_H
#define INTERFACE_H

#include "../normal_segmentation/test_lib.h"

float get_precision_input();
float get_radius_input();
float get_z_min_input();
float get_z_max_input();
int get_max_neighbs_input();
int get_file_type_input();
std::vector<float> get_xyz_input();
std::string get_import_path_input();
std::string get_export_path_input();
void clear_screen();
void invalid_input();
void success();
void failure(char const* err);
void test_menu();

#endif // INTERFACE_H

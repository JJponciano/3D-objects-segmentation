#ifndef INTERFACE_H
#define INTERFACE_H

#include "test_lib.h"

float get_radius_input();
int get_max_neighbs_input();
std::vector<float> get_xyzscale_input();
std::string get_import_path_input();
std::string get_export_path_input();
void clear_screen();
void invalid_input();
void success();
void failure(char const* err);
void test_menu();

#endif // INTERFACE_H

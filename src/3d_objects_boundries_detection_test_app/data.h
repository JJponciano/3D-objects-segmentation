#ifndef DATA_H
#define DATA_H

#include <string>

namespace data
{
    struct normal_estimation_data;
}

struct data::normal_estimation_data
{
    std::string cloud_in_path;
    std::string cloud_out_path;
    float radius;
    float max_neighbs;
    float x_scale;
    float y_scale;
    float z_scale;
    float max_fragment_depth;
};

#endif // DATA_H

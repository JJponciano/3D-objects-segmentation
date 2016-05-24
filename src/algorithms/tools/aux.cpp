#include "aux.h"


float aux::map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float aux::float_avg(std::vector<float> floats)
{
    float sum = 0;
    float float_avg = 0;

    for (auto f : floats)
    {
        sum += f;
    }

    if (floats.size() != 0)
        float_avg = sum / floats.size();

    return float_avg;
}

void aux::normal_to_rgb(pcl::PointXYZRGB *pt, vector3 normal)
{
    pt->r = normal.get_x() * 255;
    pt->g = normal.get_y() * 255;
    pt->b = normal.get_z() * 255;
}

std::vector<float> aux::spherical_coords(vector3 vect)
{
    // cartesian coordinates
    float x, y, z;

    // spherical coordinates
    float r;    // radius; the distance from the origin to the point in the YZ plane
    float theta;    // polar angle; the angle that the radius forms with the Z axis
    float phi;  // azimuth; the angle that the projection of the radius onto the XY axis forms with the X axis
    std::vector<float> spherical_coords;   // regroups the 3 other variables

    x = vect.get_x();
    y = vect.get_y();
    z = vect.get_z();

    r = std::sqrt(pow(x, 2) + pow(y, 2) +pow(z, 2));
    theta = std::acos(z / r);
    phi = std::atan(y / z);

    spherical_coords.push_back(r);
    spherical_coords.push_back(theta);
    spherical_coords.push_back(phi);

    return spherical_coords;
}

float aux::set_precision(float float_num, float precision)
{
    if (aux::cmp_floats(precision, 0.0, 0.005) || precision < 0 || ((int)precision % 10 != 0))
        throw std::logic_error("Precision cannot be 0, negative and has to be a multiple of 10.");

    return ((float)(int)(float_num * precision)) / precision;
}

bool aux::cmp_spherical_angles(std::vector<float> coords_1, std::vector<float> coords_2, float precision)
{
    if (precision == 0)
        throw std::logic_error("Angle comparison precision cannot be 0.");

    if (coords_1.size() > 2 || coords_2.size() > 2)
        throw std::logic_error("Invalid vector parameters.");

    if ((std::abs(coords_1[1] - coords_2[1]) < precision))
        return false;

    if ((std::abs(coords_1[2] - coords_2[2]) < precision))
        return false;

    return true;
}

bool aux::cmp_floats(float float_1, float float_2, float precision)
{
    if (precision == 0)
        throw std::logic_error("Float comparison precision cannot be 0.");

    if (std::abs(float_1 - float_2) < precision)
        return true;

    else
        return false;
}

vector3 aux::abs_vector(vector3 vect)
{
    vector3 abs_vector(std::abs(vect.get_x()), std::abs(vect.get_y()), std::abs(vect.get_z()));

    return abs_vector;
}

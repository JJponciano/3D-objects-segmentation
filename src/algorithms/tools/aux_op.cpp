#include "aux_op.h"

float cloud_object_segmentation::aux::set_precision(float float_num, float precision)
{
    if (cloud_object_segmentation::aux::float_cmp(precision, 0.0, 0.005) || precision < 0 || ((int)precision % 10 != 0))
        throw std::logic_error("Precision cannot be 0, negative and has to be a multiple of 10.");

    return ((float)(int)(float_num * precision)) / precision;
}

float cloud_object_segmentation::aux::map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float cloud_object_segmentation::aux::float_avg(std::vector<float> floats)
{
    float sum = 0;
    float float_avg = 0;

    for (auto f : floats)
        sum += f;

    if (floats.size() != 0)
        float_avg = sum / floats.size();

    return float_avg;
}

bool cloud_object_segmentation::aux::float_cmp(float float_1, float float_2, float precision)
{
    if (precision == 0)
        throw std::logic_error("Float comparison precision cannot be 0.");

    if (std::abs(float_1 - float_2) < precision)
        return true;

    else
        return false;
}

std::vector<float> cloud_object_segmentation::aux::spherical_coords(cloud_object_segmentation::aux::vector3 vect)
{
    // cartesian coordinates
    float x, y, z;

    // spherical coordinates
    float r;    // radius; the distance from the origin to the point in the YZ plane
    float theta;    // polar angle; the angle that the radius forms with the Z axis
    float phi;  // azimuth; the angle that the projection of the radius onto the XY axis forms with the X axis
    std::vector<float> spherical_coords;   // regroups the 3 other variables

    x = vect.x();
    y = vect.y();
    z = vect.z();

    r = std::sqrt(pow(x, 2) + pow(y, 2) +pow(z, 2));
    theta = std::acos(z / r);
    phi = std::atan(y / z);

    spherical_coords.push_back(r);
    spherical_coords.push_back(theta);
    spherical_coords.push_back(phi);

    return spherical_coords;
}

bool cloud_object_segmentation::aux::coord_cmp(std::vector<float> coords_1, std::vector<float> coords_2, float precision)
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

cloud_object_segmentation::aux::vector3 cloud_object_segmentation::aux::vect_2pts(pcl::PointXYZRGB pt_1, pcl::PointXYZRGB pt_2)
{
    cloud_object_segmentation::aux::vector3 res_vect;

    res_vect.x(pt_1.x - pt_2.x);
    res_vect.y(pt_1.y - pt_2.y);
    res_vect.z(pt_1.z - pt_2.z);

    return res_vect;
}

cloud_object_segmentation::aux::vector3 cloud_object_segmentation::aux::cross_product(cloud_object_segmentation::aux::vector3 vect_1, cloud_object_segmentation::aux::vector3 vect_2)
{
    cloud_object_segmentation::aux::vector3 cross_product;

    cross_product.x((vect_1.y() * vect_2.z()) - (vect_1.z() * vect_2.y()));
    cross_product.y((vect_1.z() * vect_2.x()) - (vect_1.x() * vect_2.z()));
    cross_product.z((vect_1.x() * vect_2.y()) - (vect_1.y() * vect_2.x()));

    return cross_product;
}

cloud_object_segmentation::aux::vector3 cloud_object_segmentation::aux::inverse(cloud_object_segmentation::aux::vector3 vect1)
{
    cloud_object_segmentation::aux::vector3 inversed_vect;

    inversed_vect.x(-vect1.x());
    inversed_vect.y(-vect1.y());
    inversed_vect.z(-vect1.z());

    return inversed_vect;
}

cloud_object_segmentation::aux::vector3 cloud_object_segmentation::aux::translate_origin(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2)
{
    cloud_object_segmentation::aux::vector3 translated_vect;

    translated_vect.x(x_2 - x_1);
    translated_vect.y(y_2 - y_1);
    translated_vect.z(z_2 - z_1);

    return translated_vect;
}

cloud_object_segmentation::aux::vector3 cloud_object_segmentation::aux::vector_avg(std::vector<cloud_object_segmentation::aux::vector3> vectors)
{
    cloud_object_segmentation::aux::vector3 vect_avg;
    float avg_x, avg_y, avg_z;

    avg_x = avg_y = avg_z = 0;

    for (auto vect : vectors)
    {
        avg_x += vect.x();
        avg_y += vect.y();
        avg_z += vect.z();
    }

    if (vectors.size() != 0)
    {
        avg_x = avg_x / vectors.size();
        avg_y = avg_y / vectors.size();
        avg_z = avg_z / vectors.size();
    }

    vect_avg.x(avg_x);
    vect_avg.y(avg_y);
    vect_avg.z(avg_z);

    return vect_avg;
}

cloud_object_segmentation::aux::vector3 cloud_object_segmentation::aux::vector_abs(cloud_object_segmentation::aux::vector3 vect)
{
    cloud_object_segmentation::aux::vector3 abs_vector;

    abs_vector.x(std::abs(vect.x()));
    abs_vector.y(std::abs(vect.y()));
    abs_vector.z(std::abs(vect.z()));

    return abs_vector;
}

cloud_object_segmentation::aux::vector3 cloud_object_segmentation::aux::normalize_normal(cloud_object_segmentation::aux::vector3 normal)
{
    cloud_object_segmentation::aux::vector3 normalized_normal;
    float length;

    if (normal.magnitude() == 0.0f)
        length = 1.0f;

    else
        length = normal.magnitude();

    normalized_normal.x(normal.x() / length);
    normalized_normal.y(normal.y() / length);
    normalized_normal.z(normal.z() / length);

    return normalized_normal;
}

void cloud_object_segmentation::aux::normal_to_rgb(pcl::PointXYZRGB *pt_ptr, cloud_object_segmentation::aux::vector3 normal)
{
    pt_ptr->r = normal.x() * 255;
    pt_ptr->g = normal.y() * 255;
    pt_ptr->b = normal.z() * 255;
}

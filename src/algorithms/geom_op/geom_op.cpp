#include "geom_op.h"

/** VECTORS **/
geom::vectors::vector3 geom::vectors::vect_2pts(pcl::PointXYZRGB pt_1, pcl::PointXYZRGB pt_2)
{
    geom::vectors::vector3 res_vect;

    res_vect.set_x(pt_1.x - pt_2.x);
    res_vect.set_y(pt_1.y - pt_2.y);
    res_vect.set_z(pt_1.z - pt_2.z);

    return res_vect;
}

geom::vectors::vector3 geom::vectors::cross_product(geom::vectors::vector3 vect_1, geom::vectors::vector3 vect_2)
{
    geom::vectors::vector3 cross_product;

    cross_product.set_x((vect_1.get_y() * vect_2.get_z())
                        - (vect_1.get_z() * vect_2.get_y()));

    cross_product.set_y((vect_1.get_z() * vect_2.get_x())
                        - (vect_1.get_x() * vect_2.get_z()));

    cross_product.set_z((vect_1.get_x() * vect_2.get_y())
                        - (vect_1.get_y() * vect_2.get_x()));

    return cross_product;
}

geom::vectors::vector3 geom::vectors::inverse(vector3 vect1)
{
    geom::vectors::vector3 inversed_vect;

    inversed_vect.set_x(-vect1.get_x());
    inversed_vect.set_y(-vect1.get_y());
    inversed_vect.set_z(-vect1.get_z());

    return inversed_vect;
}

geom::vectors::vector3 geom::vectors::translate_origin(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2)
{
    geom::vectors::vector3 translated_vect;

    translated_vect.set_x(x_2 - x_1);
    translated_vect.set_y(y_2 - y_1);
    translated_vect.set_z(z_2 - z_1);

    return translated_vect;
}

geom::vectors::vector3 geom::vectors::normalize_normal(vector3 normal)
{
    geom::vectors::vector3 normalized_normal;
    float length;

    if (normal.get_magn() == 0.0f)
        length = 1.0f;

    else
        length = normal.get_magn();

    normalized_normal.set_x(normal.get_x() / length);
    normalized_normal.set_y(normal.get_y() / length);
    normalized_normal.set_z(normal.get_z() / length);

    return normalized_normal;
}

geom::vectors::vector3 geom::vectors::vect_avg(std::vector<geom::vectors::vector3> vectors)
{
    geom::vectors::vector3 vect_avg;
    float avg_x, avg_y, avg_z;

    avg_x = avg_y = avg_z = 0;

    for (auto vect : vectors)
    {
        avg_x += vect.get_x();
        avg_y += vect.get_y();
        avg_z += vect.get_z();
    }

    if (vectors.size() != 0)
    {
        avg_x = avg_x / vectors.size();
        avg_y = avg_y / vectors.size();
        avg_z = avg_z / vectors.size();
    }

    vect_avg.set_x(avg_x);
    vect_avg.set_y(avg_y);
    vect_avg.set_z(avg_z);

    return vect_avg;
}

void geom::vectors::pcl_normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
      ne.setInputCloud(cloud);

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
      ne.setSearchMethod (tree);

      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.03);

      // Compute the features
      ne.compute (*cloud_normals);
}


/** AUX **/
float geom::aux::map(float x, float in_min, float in_max, float out_min, float out_max)
{ return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

float geom::aux::float_avg(std::vector<float> floats)
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

void geom::aux::normal_to_rgb(pcl::PointXYZRGB *pt, geom::vectors::vector3 normal)
{
    pt->r = normal.get_x() * 255;
    pt->g = normal.get_y() * 255;
    pt->b = normal.get_z() * 255;
}

std::vector<float> geom::aux::spherical_coords(vectors::vector3 vect)
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

float geom::aux::set_precision(float float_num, float precision)
{
    if (geom::aux::cmp_floats(precision, 0.0, 0.005)
            || precision < 0
            || ((int)precision % 10 != 0))
        throw std::logic_error("geom::aux::set_precision : precision cannot be 0, negative and has to be a multiple of 10.");

    else
        return ((float)(int)(float_num * precision)) / precision;
}

bool geom::aux::cmp_angles(std::vector<float> coords_1, std::vector<float> coords_2, float precision)
{
    if ((std::abs(coords_1[1] - coords_2[1]) < precision))
        return false;

    if ((std::abs(coords_1[2] - coords_2[2]) < precision))
        return false;

    return true;
}

bool geom::aux::cmp_floats(float float_1, float float_2, float precision)
{
    if (std::abs(float_1 - float_2) < precision)
        return true;

    else
        return false;
}

geom::vectors::vector3 geom::aux::abs_vector(geom::vectors::vector3 vect)
{
    geom::vectors::vector3 abs_vector(std::abs(vect.get_x()), std::abs(vect.get_y()), std::abs(vect.get_z()));

    return abs_vector;
}

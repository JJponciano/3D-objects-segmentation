#include "geom_op.h"

/** VECTORS **/
geom::vectors::vector3 geom::vectors::create_vect2p(pcl::PointXYZRGB pt1, pcl::PointXYZRGB pt2)
{
    geom::vectors::vector3 res_vect(pt1.x - pt2.x, pt1.y - pt2.y, pt1.z - pt2.z);

    return res_vect;
}

geom::vectors::vector3 geom::vectors::cross_product(geom::vectors::vector3 vect1, geom::vectors::vector3 vect2)
{
    geom::vectors::vector3 cross_product;   // the result of the cross product between the two parameter vectors

    // calculating cross product;
    cross_product.set_x((vect1.get_y() * vect2.get_z()) - (vect1.get_z() * vect2.get_y()));
    cross_product.set_y((vect1.get_z() * vect2.get_x()) - (vect1.get_x() * vect2.get_z()));
    cross_product.set_z((vect1.get_x() * vect2.get_y()) - (vect1.get_y() * vect2.get_x()));

    return cross_product;
}

geom::vectors::vector3 geom::vectors::inverse(vector3 vect1)
{
    geom::vectors::vector3 temp_vect;

    temp_vect.set_x(-vect1.get_x());
    temp_vect.set_y(-vect1.get_y());
    temp_vect.set_z(-vect1.get_z());

    return temp_vect;
}

geom::vectors::vector3 geom::vectors::translate_origin(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float x, y, z;  // translated vector's coordinates

    // calculating the coordinates of the translated vector
    x = x2 - x1;
    y = y2 - y1;
    z = z2 - z1;

    // creating and returning the new vector
    geom::vectors::vector3 temp_vect(x, y, z);
    return temp_vect;
}

geom::vectors::vector3 geom::vectors::normalize_normal(vector3 normal)
{
    // norm of the normal
    float length;

    // initializing the norm
    if (normal.get_magn() == 0.0f)
        length = 1.0f;

    else
        length = normal.get_magn();

    // normalizing normal
    geom::vectors::vector3 normalized_normal(normal.get_x() / length, normal.get_y() / length,
                                             normal.get_z() / length);

    return normalized_normal;
}

geom::vectors::vector3 geom::vectors::vect_avg(std::vector<geom::vectors::vector3> vectors)
{
    // new vector coordinates
    float x, y, z;

    // initializing vector coordinates
    x = y = z = 0;

    // summing up
    for (auto vect : vectors)
    {
        x += vect.get_x();
        y += vect.get_y();
        z += vect.get_z();
    }

    // averaging
    x = x / vectors.size();
    y = y / vectors.size();
    z = z / vectors.size();

    // defining vector
    geom::vectors::vector3 avg_vect(x, y, z);

    return avg_vect;
}

void geom::vectors::pcl_estim_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl)
{
      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
      ne.setInputCloud(pt_cl);

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
float geom::aux::float_avg(std::vector<float> floats)
{
    float sum = 0;  // sum
    float avg = 0;  // average

    for (auto f : floats)
    {
        sum += f;
    }

    avg = sum / floats.size();

    return avg;
}

void geom::aux::norm_toPtRGB(pcl::PointXYZRGB *pt, geom::vectors::vector3 normal)
{
    pt->r = normal.get_x() * 255;
    pt->g = normal.get_y() * 255;
    pt->b = normal.get_z() * 255;
}

std::vector<float> geom::aux::calc_sphcoord(vectors::vector3 vect)
{
    // cartesian coordinates
    float x, y, z;

    // spherical coordinates
    float r;    // radius; the distance from the origin to the point in the YZ plane
    float theta;    // polar angle; the angle that the radius forms with the Z axis
    float phi;  // azimuth; the angle that the projection of the radius onto the XY axis forms with the X axis
    std::vector<float> sph_coord;   // regroups the 3 other variables

    // initializing cartesian coordinates
    x = vect.get_x();
    y = vect.get_y();
    z = vect.get_z();

    // calculating r, phi and theta with a precision of 2 digits after the decimal point
    r = std::sqrt(pow(x, 2) + pow(y, 2) +pow(z, 2));
    theta = std::acos(z / r);
    phi = std::atan(y / z);

    // adding values to the vector
    sph_coord.push_back(r);
    sph_coord.push_back(theta);
    sph_coord.push_back(phi);

    return sph_coord;
}

bool geom::aux::cmp_angles(std::vector<float> coords1, std::vector<float> coords2, float eps)
{
    if ((std::abs(coords1[1]) < (std::abs(coords2[1])- eps)) || (std::abs(coords1[1]) > (std::abs(coords2[1]) + eps)))
        return false;

    if ((std::abs(coords1[2]) < (std::abs(coords2[2])- eps)) || (std::abs(coords1[2]) > (std::abs(coords2[2]) + eps)))
        return false;

    return true;
}

geom::vectors::vector3 geom::aux::abs_vector(geom::vectors::vector3 vect)
{
    geom::vectors::vector3 abs_vector(std::abs(vect.get_x()), std::abs(vect.get_y()), std::abs(vect.get_z()));

    return abs_vector;
}

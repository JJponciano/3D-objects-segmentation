#include "geom_op.h"

geom::vectors::vector3* geom::vectors::cross_product(geom::vectors::vector3 vect1, geom::vectors::vector3 vect2)
{
    geom::vectors::vector3 *cross_product;   // the result of the cross product between the two parameter vectors

    // calculating cross product
    cross_product = new geom::vectors::vector3();
    cross_product->set_x((vect1.get_y() * vect2.get_z()) - (vect1.get_z() * vect2.get_y()));
    cross_product->set_y((vect1.get_z() * vect2.get_x()) - (vect1.get_x() * vect2.get_z()));
    cross_product->set_z((vect1.get_x() * vect2.get_y()) - (vect1.get_y() * vect2.get_x()));

    return cross_product;
}

geom::vectors::vector3* geom::vectors::inverse(vector3 vect1)
{
    vector3 *temp_vect;

    temp_vect = new vector3(vect1.get_x(), vect1.get_y(), vect1.get_z());
    temp_vect->set_x(-vect1.get_x());
    temp_vect->set_y(-vect1.get_y());
    temp_vect->set_z(-vect1.get_z());

    return temp_vect;
}

geom::vectors::vector3* geom::vectors::translate_origin(float x1, float y1, float z1, float x2, float y2, float z2)
{
    vector3 *temp_vect;
    float x, y, z;  // translated vector's coordinates

    // calculating the coordinates of the translated vector
    x = x2 - x1;
    y = y2 - y1;
    z = z2 - z1;

    // creating and returning the new vector
    temp_vect = new vector3(x, y, z);
    return temp_vect;
}

geom::vectors::vector3* geom::vectors::scale(vector3 vect1, vector3 vect2)
{
    vector3 *temp_vect;

    temp_vect = new vector3(vect1.get_x(), vect1.get_y(), vect1.get_z());
    temp_vect->set_x(vect1.get_x() * vect2.get_x());
    temp_vect->set_y(vect1.get_y() * vect2.get_y());
    temp_vect->set_z(vect1.get_z() * vect2.get_z());

    return temp_vect;
}

float geom::vectors::dist(vector3 vect1, vector3 vect2)
{
    float x, y, z;  // coordinates

    x = vect1.get_x() - vect2.get_x();
    y = vect1.get_y() - vect2.get_y();
    z = vect1.get_z() - vect2.get_z();

    return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
}

geom::vectors::vector3* geom::vectors::vect_avg(std::vector<geom::vectors::vector3> vectors)
{
    // new vector coordinates
    float x, y, z;

    // averaged vector
    geom::vectors::vector3 *avg_vect;

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

    // creating new vector
    avg_vect = new geom::vectors::vector3(x, y, z);

    return avg_vect;
}

float geom::aux::calc_avg(std::vector<float> floats)
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

float geom::aux::calc_angle3p(float x1, float y1, float x2, float y2, float x3, float y3)
{
    float dist12;   // distance from point 1 to point 2
    float dist13;   // distance from point 1 to point 3
    float dist23;   // distance from point 2 to point 3
    float val_ang;  // the value of the angle calculated using arccos

    // establishing the distances between the different points
    dist12 = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    dist13 = sqrt(pow(x3 - x1, 2) + pow(y3 - y1, 2));
    dist23 = sqrt(pow(x3 - x2, 2) + pow(y3 - y2, 2));

    // calculating angle value
    val_ang = std::acos((pow(dist12, 2) + pow(dist23, 2) - pow(dist13,2)) / (2 * dist12 * dist13)) * 180/PI;
    return val_ang;
}

bool geom::aux::cmp_norm(vectors::vector3 vect1, vectors::vector3 vect2)
{
    bool similar = true;

    if ((std::abs(vect1.get_x()) > std::abs(vect2.get_x() + 35))
            || (std::abs(vect1.get_y()) > std::abs(vect2.get_y() + 35))
            || (std::abs(vect1.get_z()) > std::abs(vect2.get_z() + 35)))
        similar = false;

    return similar;
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

bool geom::aux::cmp_angles(std::vector<float> vect1, std::vector<float> vect2, float epsilon)
{
    if ((std::abs(vect1[1]) < (std::abs(vect2[1])- epsilon)) || (std::abs(vect1[1]) > (std::abs(vect2[1]) + epsilon)))
        return false;

    if ((std::abs(vect1[2]) < (std::abs(vect2[2])- epsilon)) || (std::abs(vect1[2]) > (std::abs(vect2[2]) + epsilon)))
        return false;

    return true;
}

std::vector<std::pair<pcl::PointXYZRGB *, std::vector<float>>> geom::estim_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, float range)
{
    // kd-tree used for finding neighbours
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdt;

    // auxilliary vectors for the k-tree nearest search
    std::vector<int> pointIdxRadiusSearch; // neighbours ids
    std::vector<float> pointRadiusSquaredDistance; // distances from the source to the neighbours

    // point index in the cloud
    int pt_ind = 0;

    // the vectors of which the cross product calculates the normal
    geom::vectors::vector3 *vect1;
    geom::vectors::vector3 *vect2;

    // the result of the cross product of the two previous vectors
    geom::vectors::vector3 *normal;
    geom::vectors::vector3 *translated_normal;

    // vector of vector3
    std::vector<geom::vectors::vector3> normal_toavg;

    // normal vectors average
    geom::vectors::vector3 *avged_vector;

    // theta and phi
    std::vector<float> sph_coord;

    // the resulting dictionary
    std::vector<std::pair<pcl::PointXYZRGB *, std::vector<float>>> cloud_normals;

    // cloud iterator
    pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it;

    // initializing tree
    kdt.setInputCloud(pc);

    // initializing vectors for the cross product
    vect1 = new geom::vectors::vector3();
    vect2 = new geom::vectors::vector3();

    for (cloud_it = pc->points.begin(); cloud_it < pc->points.end(); cloud_it++)
    {
        // if there are neighbours left
        if (kdt.radiusSearch(*cloud_it, range, pointIdxRadiusSearch, pointRadiusSquaredDistance, 7) > 0)
        {

            for (int pt_index = 0; pt_index < (pointIdxRadiusSearch.size() - 1); pt_index++)
            {
                if (pt_index == pointIdxRadiusSearch.size() - 2)
                {
                    // defining the first vector
                     vect1->set_x((*cloud_it).x - pc->at(pointIdxRadiusSearch[pt_index + 1]).x);
                     vect1->set_y((*cloud_it).y - pc->at(pointIdxRadiusSearch[pt_index + 1]).y);
                     vect1->set_z((*cloud_it).z - pc->at(pointIdxRadiusSearch[pt_index + 1]).z);

                     // defining the second vector
                     vect2->set_x((*cloud_it).x - pc->at(pointIdxRadiusSearch[1]).x);
                     vect2->set_y((*cloud_it).y - pc->at(pointIdxRadiusSearch[1]).y);
                     vect2->set_z((*cloud_it).z - pc->at(pointIdxRadiusSearch[1]).z);
                }


                else
                {
                    // defining the first vector
                     vect1->set_x((*cloud_it).x - pc->at(pointIdxRadiusSearch[pt_index + 1]).x);
                     vect1->set_y((*cloud_it).y - pc->at(pointIdxRadiusSearch[pt_index + 1]).y);
                     vect1->set_z((*cloud_it).z - pc->at(pointIdxRadiusSearch[pt_index + 1]).z);

                     // defining the second vector
                     vect2->set_x((*cloud_it).x - pc->at(pointIdxRadiusSearch[pt_index + 2]).x);
                     vect2->set_y((*cloud_it).y - pc->at(pointIdxRadiusSearch[pt_index + 2]).y);
                     vect2->set_z((*cloud_it).z - pc->at(pointIdxRadiusSearch[pt_index + 2]).z);
                }

                // calculating normal
                normal = geom::vectors::cross_product(*vect1, *vect2);
                translated_normal = geom::vectors::translate_origin((*cloud_it).x, (*cloud_it).y, (*cloud_it).z, normal->get_x(), normal->get_y(), normal->get_z());

                normal_toavg.push_back(*translated_normal);
            }

            // averaging vectors
            avged_vector = geom::vectors::vect_avg(normal_toavg);

            // calculating phi and theta for the average vector
            sph_coord = geom::aux::calc_sphcoord(*avged_vector);

            // adding the new entry to the dictionary
            cloud_normals.push_back(std::pair<pcl::PointXYZRGB *, std::vector<float>>(&(*cloud_it), sph_coord));

            // resetting vectors
            sph_coord.clear();
            normal_toavg.clear();

            // resetting neighbours
            pointIdxRadiusSearch.clear();
            pointRadiusSquaredDistance.clear();
        }
    }

    return cloud_normals;
}

void pcl_estim_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc)
{
      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
      ne.setInputCloud(pc);

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

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

std::vector<float> geom::aux::calc_sphcoord(vectors::vector3 vect)
{
    // spherical coordinates
    float r;    // radius; the distance from the origin to the point in the YZ plane
    float theta;    // polar angle; the angle that the radius forms with the Z axis
    float phi;  // azimuth; the angle that the projection of the radius onto the XY axis forms with the X axis
    std::vector<float> sph_coord;   // regroups the 3 other variables

    // calculating r, phi and theta with a precision of 2 digits after the decimal point
    r = std::sqrt(pow(vect.get_x(), 2) + pow(vect.get_y(), 2) +pow(vect.get_z(), 2));
    theta = std::acos(vect.get_z() / r);
    phi = std::atan(vect.get_y() / vect.get_z());

    // adding values to the vector
    sph_coord.push_back(r);
    sph_coord.push_back(theta);
    sph_coord.push_back(phi);

    return sph_coord;
}

std::vector<std::pair<pcl::PointXYZRGB *, std::string>> geom::estim_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, int range)
{
    // kd-tree used for finding neighbours
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdt;

    // auxilliary vectors for the k-tree nearest search
    std::vector<int> pointIdxNKNSearch(range); // neighbours ids
    std::vector<float> pointNKNSquaredDistance(range); // distances from the source to the neighbours

    // the vectors of which the cross product calculates the normal
    geom::vectors::vector3 *vect1;
    geom::vectors::vector3 *vect2;

    // the result of the cross product of the two previous vectors
    geom::vectors::vector3 *normal;
    geom::vectors::vector3 *translated_normal;

    // spherical coordinates of the point found at the arrow of a vector
    std::vector<float> norm_sphcoord;

    // calculating the normal vector to the planes that the currently treated point
    // makes with its n other neighbours
    std::vector<float> r_vals;
    std::vector<float> theta_vals;
    std::vector<float> phi_vals;

    // averaging the obtained values
    float theta_avg;
    float phi_avg;


    // float to string conversion
    std::stringstream etrval_ss;
    std::string entry_value;  // the value that corresponds to the key given by the point

    // the resulting dictionary
    std::vector<std::pair<pcl::PointXYZRGB *, std::string>> cloud_normals;

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
        if (kdt.nearestKSearch(*cloud_it, range, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {

            for (int pt_index = 0; pt_index < (pointIdxNKNSearch.size() - 1); pt_index++)
            {
                if (pt_index == pointIdxNKNSearch.size() - 2)
                {
                    // defining the first vector
                     vect1->set_x((*cloud_it).x - pc->at(pointIdxNKNSearch[pt_index + 1]).x);
                     vect1->set_y((*cloud_it).y - pc->at(pointIdxNKNSearch[pt_index + 1]).y);
                     vect1->set_z((*cloud_it).z - pc->at(pointIdxNKNSearch[pt_index + 1]).z);

                     // defining the second vector
                     vect2->set_x((*cloud_it).x - pc->at(pointIdxNKNSearch[1]).x);
                     vect2->set_y((*cloud_it).y - pc->at(pointIdxNKNSearch[1]).y);
                     vect2->set_z((*cloud_it).z - pc->at(pointIdxNKNSearch[1]).z);
                }


                else
                {
                    // defining the first vector
                     vect1->set_x((*cloud_it).x - pc->at(pointIdxNKNSearch[pt_index + 1]).x);
                     vect1->set_y((*cloud_it).y - pc->at(pointIdxNKNSearch[pt_index + 1]).y);
                     vect1->set_z((*cloud_it).z - pc->at(pointIdxNKNSearch[pt_index + 1]).z);

                     // defining the second vector
                     vect2->set_x((*cloud_it).x - pc->at(pointIdxNKNSearch[pt_index + 2]).x);
                     vect2->set_y((*cloud_it).y - pc->at(pointIdxNKNSearch[pt_index + 2]).y);
                     vect2->set_z((*cloud_it).z - pc->at(pointIdxNKNSearch[pt_index + 2]).z);
                }

                // calculating normal
                normal = geom::vectors::cross_product(*vect1, *vect2);
                translated_normal = geom::vectors::translate_origin((*cloud_it).x, (*cloud_it).y, (*cloud_it).z, normal->get_x(), normal->get_y(), normal->get_z());

                // calculating spherical coordinates
                norm_sphcoord = geom::aux::calc_sphcoord(*translated_normal);

                // storing those values
                r_vals.push_back(norm_sphcoord[0]);
                theta_vals.push_back(norm_sphcoord[1]);
                phi_vals.push_back(norm_sphcoord[2]);
            }

            // calculating an average of the obtained spherical coordinates
            theta_avg = geom::aux::calc_avg(theta_vals);
            phi_avg = geom::aux::calc_avg(phi_vals);

            // converting phi and theta and creating an entry value
            // lexical_cast as used below performs the float to string cast in an efficient manner
            etrval_ss << std::fixed << std::setprecision(0) << theta_avg << phi_avg;
            entry_value = etrval_ss.str();
            etrval_ss.str(std::string());
            etrval_ss.flush();

            // adding the new entry to the dictionary
            cloud_normals.push_back(std::pair<pcl::PointXYZRGB *, std::string>(&(*cloud_it), entry_value));

            // resetting vectors
            r_vals.clear();
            theta_vals.clear();
            phi_vals.clear();
            norm_sphcoord.clear();

            // resetting neighbours
            pointIdxNKNSearch.clear();
            pointNKNSquaredDistance.clear();
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

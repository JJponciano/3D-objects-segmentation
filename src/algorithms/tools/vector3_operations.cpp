#include "vector3_operations.h"

vector3 vector3_operations::vect_2pts(pcl::PointXYZRGB pt_1, pcl::PointXYZRGB pt_2)
{
    vector3 res_vect;

    res_vect.set_x(pt_1.x - pt_2.x);
    res_vect.set_y(pt_1.y - pt_2.y);
    res_vect.set_z(pt_1.z - pt_2.z);

    return res_vect;
}

vector3 vector3_operations::cross_product(vector3 vect_1, vector3 vect_2)
{
    vector3 cross_product;

    cross_product.set_x((vect_1.get_y() * vect_2.get_z())
                        - (vect_1.get_z() * vect_2.get_y()));

    cross_product.set_y((vect_1.get_z() * vect_2.get_x())
                        - (vect_1.get_x() * vect_2.get_z()));

    cross_product.set_z((vect_1.get_x() * vect_2.get_y())
                        - (vect_1.get_y() * vect_2.get_x()));

    return cross_product;
}

vector3 vector3_operations::inverse(vector3 vect1)
{
    vector3 inversed_vect;

    inversed_vect.set_x(-vect1.get_x());
    inversed_vect.set_y(-vect1.get_y());
    inversed_vect.set_z(-vect1.get_z());

    return inversed_vect;
}

vector3 vector3_operations::translate_origin(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2)
{
    vector3 translated_vect;

    translated_vect.set_x(x_2 - x_1);
    translated_vect.set_y(y_2 - y_1);
    translated_vect.set_z(z_2 - z_1);

    return translated_vect;
}

vector3 vector3_operations::normalize_normal(vector3 normal)
{
    vector3 normalized_normal;
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

vector3 vector3_operations::vect_avg(std::vector<vector3> vectors)
{
    vector3 vect_avg;
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

void vector3_operations::pcl_normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
      if (!cloud_ptr)
      {
        QString err_msg = "vector3_operations::pcl_normal_estimation : Cloud pointer cannot be null.";

        throw err_msg;
      }

      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
      ne.setInputCloud(cloud_ptr);

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_ptr (new pcl::search::KdTree<pcl::PointXYZRGB> ());
      ne.setSearchMethod (tree_ptr);

      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.03);

      // Compute the features
      ne.compute (*cloud_normals_ptr);
}

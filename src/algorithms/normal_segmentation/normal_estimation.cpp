#include "normal_estimation.h"

void normal_segmentation::estimate_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr,
                                            float radius, int max_neighbs)
{
    if (!cloud_ptr)
        throw invalid_cloud_pointer();

    if (aux::float_cmp(radius, 0.00, 0.005))
        throw std::logic_error("Invalid radius value.");

    if (aux::float_cmp(max_neighbs, 0.00, 0.005))
        throw std::logic_error("Invalid max neighbours value.");

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdt; // kd-tree used for finding neighbours

    kdt.setInputCloud(cloud_ptr);

    for (auto cloud_it = cloud_ptr->begin(); cloud_it < cloud_ptr->end(); cloud_it++)
    {
        std::vector<int> pt_ids; // neighbours' ids
        std::vector<float> pt_sq_dist; // distances from the source to the neighbours

        // if there are neighbours left
        if (kdt.radiusSearch(*cloud_it, radius, pt_ids, pt_sq_dist, max_neighbs) > 0)
        {
            std::vector<aux::vector3> vects_to_avg; // vect_or average used for estimating normal;

            for (size_t pt_index = 0; pt_index < (pt_ids.size() - 1); pt_index++)
            {
                aux::vector3 vect_1 = aux::vect_2pts(*cloud_it,
                                                     cloud_ptr->points[pt_ids[pt_index + 1]]);
                aux::vector3 vect_2;

                // defining the second vect_or; making sure there is no 'out of bounds' error
                if (pt_index == pt_ids.size() - 2)
                    vect_2 = aux::vect_2pts(*cloud_it, cloud_ptr->points[pt_ids[1]]);

                else
                    vect_2 = aux::vect_2pts(*cloud_it, cloud_ptr->points[pt_ids[pt_index + 2]]);

                vects_to_avg.push_back(aux::vector_abs(aux::cross_product(vect_1, vect_2)));
            }

            // calculating the normal and coloring the point based on its coordinates
            aux::vector3 normal = aux::normalize_normal(aux::vector_avg(vects_to_avg));
            aux::normal_to_rgb(&(*cloud_it), normal);

            vects_to_avg.clear();
            vects_to_avg.shrink_to_fit();
        }

        pt_ids.clear();
        pt_sq_dist.clear();
        pt_ids.shrink_to_fit();
        pt_sq_dist.shrink_to_fit();
    }
}

void normal_segmentation::estimate_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
      if (!cloud_ptr)
        throw invalid_cloud_pointer();

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

#include "normal_estimation.h"

void normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, float radius, int max_neighbs)
{
    if (!cloud_ptr)
    {
        throw invalid_cloud_pointer();
    }

    if (aux::float_cmp(radius, 0.00, 0.005))
    {
        throw std::logic_error("Invalid radius value.");
    }

    if (aux::float_cmp(max_neighbs, 0.00, 0.005))
    {
        throw std::logic_error("Invalid max neighbours value.");
    }

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdt; // kd-tree used for finding neighbours

    std::vector<int> pointIdxRadiusSearch; // neighbours' ids
    std::vector<float> pointRadiusSquaredDistance; // distances from the source to the neighbours

    // the vect_ors of which the cross product calculates the normal
    vector3 vect_1;
    vector3 vect_2;
    vector3 normal;

    std::vector<vector3> vects_to_avg; // vect_or average used for estimating normal

    kdt.setInputCloud(cloud_ptr);

    for (unsigned int cloud_it = 0; cloud_it < cloud_ptr->points.size(); cloud_it++)
    {
        // if there are neighbours left
        if (kdt.radiusSearch(cloud_ptr->points[cloud_it], radius, pointIdxRadiusSearch,
                             pointRadiusSquaredDistance, max_neighbs) > 0)
        {

            for (unsigned int pt_index = 0; pt_index < (pointIdxRadiusSearch.size() - 1); pt_index++)
            {
                vect_1 = vector3_operations::vect_2pts((cloud_ptr->points[cloud_it]),
                                                 cloud_ptr->points[pointIdxRadiusSearch[pt_index + 1]]);

                // defining the second vect_or; making sure there is no 'out of bounds' error
                if (pt_index == pointIdxRadiusSearch.size() - 2)
                {
                    vect_2 = vector3_operations::vect_2pts((cloud_ptr->points[cloud_it]),
                                                      cloud_ptr->points[pointIdxRadiusSearch[1]]);
                }


                else
                {
                    vect_2 = vector3_operations::vect_2pts((cloud_ptr->points[cloud_it]),
                                                      cloud_ptr->points[pointIdxRadiusSearch[pt_index + 2]]);
                }

                vects_to_avg.push_back(aux::vector_abs(vector3_operations::cross_product(vect_1, vect_2)));
            }

            // calculating the normal and coloring the point based on its coordinates
            normal = vector3_operations::normalize_normal(vector3_operations::vect_avg(vects_to_avg));
            aux::normal_to_rgb(&(cloud_ptr->points[cloud_it]), normal);

            // freeing memory
            vects_to_avg.clear();
            pointIdxRadiusSearch.clear();
            pointRadiusSquaredDistance.clear();
            vects_to_avg.shrink_to_fit();
            pointIdxRadiusSearch.shrink_to_fit();
            pointRadiusSquaredDistance.shrink_to_fit();
        }
    }
}

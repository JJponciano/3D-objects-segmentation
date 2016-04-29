#include "normal_estimation.h"

void normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl, float radius, int max_neighbs)
{
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdt; // kd-tree used for finding neighbours

    unsigned int cloud_it;  // cloud iterator

    std::vector<int> pointIdxRadiusSearch; // neighbours' ids
    std::vector<float> pointRadiusSquaredDistance; // distances from the source to the neighbours

    // the vect_ors of which the cross product calculates the normal
    geom::vectors::vector3 vect_1;
    geom::vectors::vector3 vect_2;
    geom::vectors::vector3 normal;

    std::vector<geom::vectors::vector3> vects_to_avg; // vect_or average used for estimating normal

    kdt.setInputCloud(pt_cl);

    for (cloud_it = 0; cloud_it < pt_cl->points.size(); cloud_it++)
    {
        // if there are neighbours left
        if (kdt.radiusSearch(pt_cl->points[cloud_it], radius, pointIdxRadiusSearch,
                             pointRadiusSquaredDistance, max_neighbs) > 0)
        {

            for (unsigned int pt_index = 0; pt_index < (pointIdxRadiusSearch.size() - 1); pt_index++)
            {
                vect_1 = geom::vectors::vect_2pts((pt_cl->points[cloud_it]),
                                                 pt_cl->points[pointIdxRadiusSearch[pt_index + 1]]);

                // defining the second vect_or; making sure there is no 'out of bounds' error
                if (pt_index == pointIdxRadiusSearch.size() - 2)
                    vect_2 = geom::vectors::vect_2pts((pt_cl->points[cloud_it]),
                                                      pt_cl->points[pointIdxRadiusSearch[1]]);


                else
                    vect_2 = geom::vectors::vect_2pts((pt_cl->points[cloud_it]),
                                                      pt_cl->points[pointIdxRadiusSearch[pt_index + 2]]);

                vects_to_avg.push_back(geom::aux::abs_vector(geom::vectors::cross_product(vect_1, vect_2)));
            }

            // calculating the normal and coloring the point based on its coordinates
            normal = geom::vectors::normalize_normal(geom::vectors::vect_avg(vects_to_avg));
            geom::aux::normal_to_rgb(&(pt_cl->points[cloud_it]), normal);

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

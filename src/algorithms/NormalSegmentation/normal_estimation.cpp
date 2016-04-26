#include "normal_estimation.h"

void estim_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl, float radius, int max_neighbs)
{
    // kd-tree used for finding neighbours
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdt;

    // initializing tree
    kdt.setInputCloud(pt_cl);

    // cloud iterator
    unsigned int cloud_it;

    // auxilliary vectors for the k-tree nearest search
    std::vector<int> pointIdxRadiusSearch; // neighbours ids
    std::vector<float> pointRadiusSquaredDistance; // distances from the source to the neighbours

    // the vectors of which the cross product calculates the normal
    geom::vectors::vector3 vect1;
    geom::vectors::vector3 vect2;
    geom::vectors::vector3 normal;

    // vectors to average
    std::vector<geom::vectors::vector3> vct_toavg;

    for (cloud_it = 0; cloud_it < pt_cl->points.size(); cloud_it++)
    {
        // if there are neighbours left
        if (kdt.radiusSearch(pt_cl->points[cloud_it], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, max_neighbs) > 0)
        {

            for (int pt_index = 0; pt_index < (pointIdxRadiusSearch.size() - 1); pt_index++)
            {
                // defining the first vector
                vect1 = geom::vectors::create_vect2p((pt_cl->points[cloud_it]), pt_cl->points[pointIdxRadiusSearch[pt_index + 1]]);

                // defining the second vector; making sure there is no 'out of bounds' error
                if (pt_index == pointIdxRadiusSearch.size() - 2)
                    vect2 = geom::vectors::create_vect2p((pt_cl->points[cloud_it]), pt_cl->points[pointIdxRadiusSearch[1]]);


                else
                    vect2 = geom::vectors::create_vect2p((pt_cl->points[cloud_it]), pt_cl->points[pointIdxRadiusSearch[pt_index + 2]]);

                // adding the cross product of the two previous vectors to our list
                vct_toavg.push_back(geom::aux::abs_vector(geom::vectors::cross_product(vect1, vect2)));
            }

            // calculating the normal and coloring the point
            normal = geom::vectors::normalize_normal(geom::vectors::vect_avg(vct_toavg));
            geom::aux::norm_toPtRGB(&(pt_cl->points[cloud_it]), normal);

            // freeing memory
            vct_toavg.clear();
            pointIdxRadiusSearch.clear();
            pointRadiusSquaredDistance.clear();
            vct_toavg.shrink_to_fit();
            pointIdxRadiusSearch.shrink_to_fit();
            pointRadiusSquaredDistance.shrink_to_fit();
        }
    }
}

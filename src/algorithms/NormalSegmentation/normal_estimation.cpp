#include "normal_estimation.h"

void estim_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl, float radius)
{
    // kd-tree used for finding neighbours
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdt;

    // auxilliary vectors for the k-tree nearest search
    std::vector<int> pointIdxRadiusSearch; // neighbours ids
    std::vector<float> pointRadiusSquaredDistance; // distances from the source to the neighbours

    // the vectors of which the cross product calculates the normal
    geom::vectors::vector3 *vect1;
    geom::vectors::vector3 *vect2;
    geom::vectors::vector3 *cross_prod;
    geom::vectors::vector3 *abs_cross_prod;

    // vectors to average
    std::vector<geom::vectors::vector3> vct_toavg;
    geom::vectors::vector3 *vct_avg;

    // the result of the cross product of the two previous vectors
    geom::vectors::vector3 *normal;

    // cloud iterator
    pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it;

    // initializing tree
    kdt.setInputCloud(pt_cl);

    for (cloud_it = pt_cl->points.begin(); cloud_it < pt_cl->points.end(); cloud_it++)
    {
        // if there are neighbours left
        if (kdt.radiusSearch(*cloud_it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, 100) > 0)
        {

            for (int pt_index = 0; pt_index < (pointIdxRadiusSearch.size() - 1); pt_index++)
            {
                // defining the first vector
                vect1 = geom::vectors::create_vect2p((*cloud_it), pt_cl->points[pointIdxRadiusSearch[pt_index + 1]]);

                // defining the second vector; making sure there is no 'out of bounds' error
                if (pt_index == pointIdxRadiusSearch.size() - 2)
                    vect2 = geom::vectors::create_vect2p((*cloud_it), pt_cl->points[pointIdxRadiusSearch[1]]);

                else
                    vect2 = geom::vectors::create_vect2p((*cloud_it), pt_cl->points[pointIdxRadiusSearch[pt_index + 2]]);

                // adding the cross product of the two previous vectors to our list
                cross_prod = geom::vectors::cross_product(*vect1, *vect2);
                abs_cross_prod = geom::aux::abs_vector(*cross_prod);
                vct_toavg.push_back(*abs_cross_prod);

                // freeing memory
                delete cross_prod;
                delete abs_cross_prod;
                delete vect1;
                delete vect2;
            }

            // calculating the normalized normal
            vct_avg = geom::vectors::vect_avg(vct_toavg);
            normal = geom::vectors::normalize_normal(*vct_avg);

            // calculating point colors and adding it to the result list
            geom::aux::norm_toPtRGB(&(*(cloud_it)), *normal);

            // freeing memory
            delete vct_avg;
            delete normal;

            // clearing vectors
            vct_toavg.clear();
            pointIdxRadiusSearch.clear();
            pointRadiusSquaredDistance.clear();

            // shrinking vectors
            vct_toavg.shrink_to_fit();
            pointIdxRadiusSearch.shrink_to_fit();
            pointRadiusSquaredDistance.shrink_to_fit();
        }
    }
}

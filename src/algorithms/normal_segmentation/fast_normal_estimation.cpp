#include "fast_normal_estimation.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr fast_normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, int max_neighbs,
                                                    float radius, float x_scale, float y_scale, float z_scale, float max_fragment_depth)
{
    // the cloud colored by its normal vectors; return value
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_ptr;

    float max_scaled_fragment_depth = max_fragment_depth / y_scale;

    cloud_manip::scale_cloud(cloud_ptr, x_scale, y_scale, z_scale); // scaling cloud

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_fragments =
            cloud_manip::fragment_cloud(cloud_ptr, max_scaled_fragment_depth); // fragmenting cloud for less execution time

    // estimating the normals for each cloud fragment in parallel
    #pragma omp parallel for schedule(static)
    for (unsigned int i = 0; i < cloud_fragments.size(); i++)
    {
        normal_estimation(cloud_fragments[i], radius, max_neighbs);
    }

    colored_cloud_ptr = cloud_manip::merge_clouds(cloud_fragments); // merging fragments to build original cloud

    cloud_manip::scale_cloud(colored_cloud_ptr, (1.0/x_scale), (1.0/y_scale), (1.0/z_scale));    // restoring widop scale

    return colored_cloud_ptr;
}

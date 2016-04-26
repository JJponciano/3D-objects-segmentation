#include "eff_norm_est.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr eff_norm_est(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl, float radius, int max_neighbs, float y_scale)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;   // the cloud colored by its normal vectors; return value

    cloud_manip::widop_to_cloud(pt_cl, y_scale); // scaling cloud

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_fragm =
            cloud_manip::fragment_cloud(pt_cl, y_scale); // fragmenting cloud for faster treatment

    // estimating the normals for each cloud fragment in parallel
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < cloud_fragm.size(); i++)
    {
        estim_normals(cloud_fragm[i], radius, max_neighbs);
    }

    colored_cloud = cloud_manip::merge_clouds(cloud_fragm); // merging fragments to build original cloud
    cloud_manip::cloud_to_widop(colored_cloud, y_scale);    // restoring widop scale

    return colored_cloud;
}

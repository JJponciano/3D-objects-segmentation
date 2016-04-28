#include "cloud_manip.h"

void cloud_manip::widop_to_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl, float y_scale)
{
    // iterator
    pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it;

    // scaling cloud
    for (cloud_it = pt_cl->points.begin(); cloud_it < pt_cl->points.end(); cloud_it++)
        (*cloud_it).y /= y_scale;
}

void cloud_manip::cloud_to_widop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl, float y_scale)
{
    // iterator
    pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it;

    // scaling cloud
    for (cloud_it = pt_cl->points.begin(); cloud_it < pt_cl->points.end(); cloud_it++)
        (*cloud_it).y *= y_scale;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_manip::fragment_cloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl, float y_scale)
{
    const float range = 500 / y_scale;
    float curr_y = FLT_MAX;    // the range will be in function of y
    int cloud_it;   // cloud iterator
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cl_fragm;

    for (cloud_it = 0; cloud_it < pt_cl->points.size(); cloud_it++)
    {
        // end of a fragment
        if ((pt_cl->points[cloud_it].y > (curr_y + range))
                || (pt_cl->points[cloud_it].y < (curr_y - range)) )
        {
            curr_y = pt_cl->points[cloud_it].y;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            cl_fragm.push_back(new_cloud);
        }

        // filling current cloud
        else
            (cl_fragm.back())->points.push_back(pt_cl->points[cloud_it]);
    }

    return cl_fragm;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_manip::merge_clouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fragments)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; i < fragments.size(); i++)
    {
        for (int j = 0; j < fragments[i]->points.size(); j++)
        {
            pt_cl->points.push_back(fragments[i]->points[j]);
        }
    }

    return pt_cl;
}

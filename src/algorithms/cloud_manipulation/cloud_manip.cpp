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
    size_t cloud_it;   // cloud iterator
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

    for (size_t i = 0; i < fragments.size(); i++)
    {
        for (size_t j = 0; j < fragments[i]->points.size(); j++)
        {
            pt_cl->points.push_back(fragments[i]->points[j]);
        }
    }

    return pt_cl;
}

void cloud_manip::convertXYZRGBToBool(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB, pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool)
{
    // Make sure that both cloud are of same size and avoid segmentation fault
    cloud_bool->width = cloud_RGB->width;
    cloud_bool->height = cloud_RGB->height;
    cloud_bool->resize(cloud_bool->width * cloud_bool->height);
    // Copies each point into the other cloud
    for(size_t i=0; i<cloud_RGB->points.size(); i++)
    {
        cloud_bool->points[i].x = cloud_RGB->points[i].x;
        cloud_bool->points[i].y = cloud_RGB->points[i].y;
        cloud_bool->points[i].z = cloud_RGB->points[i].z;
        cloud_bool->points[i].rgb = cloud_RGB->points[i].rgb;
        cloud_bool->points[i].setVisited(false);
    }
}

void cloud_manip::convertBoolToXYZRGB(pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB)
{
    cloud_RGB->width = cloud_bool->width;
    cloud_RGB->height = cloud_bool->height;
    cloud_RGB->resize(cloud_RGB->width * cloud_RGB->height);
    for(size_t i=0; i<cloud_bool->points.size(); i++)
    {
        cloud_RGB->points[i].x = cloud_bool->points[i].x;
        cloud_RGB->points[i].y = cloud_bool->points[i].y;
        cloud_RGB->points[i].z = cloud_bool->points[i].z;
        cloud_RGB->points[i].rgb = cloud_bool->points[i].rgb;
    }
}

void cloud_manip::giveRandomColorToCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    uint8_t r=rand()%255,g=rand()%255,b=rand()%255;
    uint32_t rgb = (uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b;

    pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it;
    for(cloud_it=cloud->begin(); cloud_it!=cloud->end(); cloud_it++)
    {
        (*cloud_it).rgb = rgb;
    }
}

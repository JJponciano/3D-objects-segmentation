#include "cloud_manip.h"

void cloud_manip::copy_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr dest)
{
    for (unsigned int cloud_it = 0; cloud_it < src->points.size(); cloud_it++)
    {
        dest->points.push_back(src->points[cloud_it]);
    }
}

void cloud_manip::scale_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl,
                                 float x_scale,
                                 float y_scale,
                                 float z_scale,
                                 float precision)
{
    if (geom::aux::cmp_floats(x_scale, 0.00, precision)
            || geom::aux::cmp_floats(y_scale, 0.00, precision)
            || geom::aux::cmp_floats(z_scale, 0.00, precision))
        throw std::logic_error("cloud_manip::scale_cloud : Division by 0 is not possible.");

    else
    {
        for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = pt_cl->points.begin();
             cloud_it < pt_cl->points.end(); cloud_it++)
        {
            (*cloud_it).x /= x_scale;
            (*cloud_it).y /= y_scale;
            (*cloud_it).z /= z_scale;
        }
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_manip::fragment_cloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl, float y_scale, float precision)
{
    if (!geom::aux::cmp_floats(y_scale, 0.00, precision))
    {
        const float range = 500 / y_scale;
        float curr_y = FLT_MAX;    // the range will be in function of y
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cl_fragm;

        for (unsigned int cloud_it = 0; cloud_it < pt_cl->points.size(); cloud_it++)
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

    else
        throw std::logic_error("cloud_manip::fragment_cloud : Division by 0 is not possible.");
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_manip::crop_cloud(
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl,
                       float x_thresh, float y_thresh,
                       float z_thresh, float precision)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (unsigned int cloud_it = 0; cloud_it < pt_cl->points.size(); cloud_it++)
    {
        bool remove_point = false;

        if ((std::abs(pt_cl->points[cloud_it].x) > std::abs(x_thresh))
                && (!geom::aux::cmp_floats(x_thresh, 0, precision)))
            remove_point = true;

        if ((std::abs(pt_cl->points[cloud_it].y) > std::abs(y_thresh))
                && (!geom::aux::cmp_floats(y_thresh, 0, precision)))
            remove_point = true;

        if (std::abs(pt_cl->points[cloud_it].z) > std::abs(z_thresh)
                && (!geom::aux::cmp_floats(z_thresh, 0, precision)))
            remove_point = true;

        if (!remove_point)
            cropped_cloud->points.push_back(pt_cl->points[cloud_it]);
    }

    return cropped_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_manip::merge_clouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fragments)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (unsigned int i = 0; i < fragments.size(); i++)
    {
        for (unsigned int j = 0; j < fragments[i]->points.size(); j++)
        {
            pt_cl->points.push_back(fragments[i]->points[j]);
        }
    }

    return pt_cl;
}

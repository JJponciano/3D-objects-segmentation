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

std::vector<point_xy_greyscale> cloud_manip::color_to_greyscale(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl,
        float min_z, float max_z)
{
    std::vector<point_xy_greyscale> gs_points;

    for (unsigned int cloud_it = 0; cloud_it < pt_cl->points.size(); cloud_it++)
    {
        point_xy_greyscale pt_xy_gs;
        float greyscale = geom::aux::map(pt_cl->points[cloud_it].z,
                                       min_z, max_z,
                                       0.0, 255.0);

        pt_xy_gs.x = pt_cl->points[cloud_it].x;
        pt_xy_gs.y = pt_cl->points[cloud_it].y;
        pt_xy_gs.greyscale((unsigned short)greyscale);
        gs_points.push_back(pt_xy_gs);
    }

    return gs_points;
}


void cloud_manip::cloud_homogenization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl,
                                       short epsilon)
{
    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = pt_cl->begin();
     cloud_it < pt_cl->end(); cloud_it++)
    {
        short r_times_epsilon = (short)(*cloud_it).r / epsilon;
        short g_times_epsilon = (short)(*cloud_it).g / epsilon;
        short b_times_epsilon = (short)(*cloud_it).b / epsilon;

        if ((r_times_epsilon * epsilon) > 255)
            (*cloud_it).r = 255;

        else
            (*cloud_it).r = r_times_epsilon * epsilon;

        if ((g_times_epsilon * epsilon) > 255)
            (*cloud_it).g = 255;

        else
            (*cloud_it).g = g_times_epsilon * epsilon;

        if ((b_times_epsilon * epsilon) > 255)
            (*cloud_it).b = 255;

        else
            (*cloud_it).b = b_times_epsilon * epsilon;
    }
}

void cloud_manip::convertBoolToXYZRGB(pcl::PointCloud<clstr::point_clstr>::Ptr cloud_bool, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB)
{
    cloud_RGB->width = cloud_bool->width;
    cloud_RGB->height = cloud_bool->height;
    cloud_RGB->resize(cloud_RGB->width * cloud_RGB->height);
    for(size_t i=0; i<cloud_bool->points.size(); i++)
    {
        cloud_RGB->points[i].x = cloud_bool->points[i].x;
        cloud_RGB->points[i].y = cloud_bool->points[i].y;
        cloud_RGB->points[i].z = cloud_bool->points[i].z;
        cloud_RGB->points[i].r = cloud_bool->points[i].r;
	cloud_RGB->points[i].g = cloud_bool->points[i].g;
	cloud_RGB->points[i].b = cloud_bool->points[i].b;
    }
}

void cloud_manip::convertXYZRGBToBool(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB, pcl::PointCloud<clstr::point_clstr>::Ptr cloud_bool)
{
    cloud_bool->width = cloud_RGB->width;
    cloud_bool->height = cloud_RGB->height;
    cloud_bool->resize(cloud_bool->width * cloud_bool->height);
    for(size_t i=0; i<cloud_RGB->points.size(); i++)
    {
        cloud_bool->points[i].x = cloud_RGB->points[i].x;
        cloud_bool->points[i].y = cloud_RGB->points[i].y;
        cloud_bool->points[i].z = cloud_RGB->points[i].z;
        cloud_bool->points[i].r = cloud_RGB->points[i].r;
	cloud_bool->points[i].g = cloud_RGB->points[i].g;
	cloud_bool->points[i].b = cloud_RGB->points[i].b;
    }
}

void cloud_manip::giveRandomColorToCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    uint8_t r=rand()%255,g=rand()%255,b=rand()%255;

    pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it;
    for(cloud_it=cloud->begin(); cloud_it!=cloud->end(); cloud_it++)
    {
        (*cloud_it).r = r;
        (*cloud_it).g = g;
        (*cloud_it).b = b;
    }
}

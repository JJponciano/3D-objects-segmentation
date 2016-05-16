#include "cloud_manip.h"

std::vector<float> cloud_manip::cloud_x_coords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
    std::vector<float> x_coords;

    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = cloud_ptr->begin();
         cloud_it < cloud_ptr->end(); cloud_it++)
        x_coords.push_back((float)(cloud_it->x));

    return x_coords;
}

std::vector<float> cloud_manip::cloud_y_coords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
    std::vector<float> y_coords;

    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = cloud_ptr->begin();
         cloud_it < cloud_ptr->end(); cloud_it++)
        y_coords.push_back((float)(cloud_it->y));

    return y_coords;
}

std::vector<float> cloud_manip::cloud_z_coords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
    std::vector<float> z_coords;

    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = cloud_ptr->begin();
         cloud_it < cloud_ptr->end(); cloud_it++)
        z_coords.push_back((float)(cloud_it->z));

    return z_coords;
}

void cloud_manip::copy_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_ptr,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr dest_ptr)
{
    for (unsigned int cloud_it = 0; cloud_it < src_ptr->points.size(); cloud_it++)
    {
        dest_ptr->points.push_back(src_ptr->points[cloud_it]);
    }
}

void cloud_manip::scale_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, float x_scale, float y_scale,
                                 float z_scale, float precision)
{
    if (geom::aux::cmp_floats(x_scale, 0.00, precision)
            || geom::aux::cmp_floats(y_scale, 0.00, precision)
            || geom::aux::cmp_floats(z_scale, 0.00, precision))
        throw std::logic_error("cloud_manip::scale_cloud : Multiplying by 0 will destroy the cloud.");

    else
    {
        for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = cloud_ptr->points.begin();
             cloud_it < cloud_ptr->points.end(); cloud_it++)
        {
            (*cloud_it).x *= x_scale;
            (*cloud_it).y *= y_scale;
            (*cloud_it).z *= z_scale;
        }
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_manip::fragment_cloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, float max_scaled_fragment_depth, float precision)
{
    if (!geom::aux::cmp_floats(max_scaled_fragment_depth, 0.00, precision))
    {
        float curr_depth = FLT_MAX;
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_fragments;

        for (unsigned int cloud_it = 0; cloud_it < cloud_ptr->points.size(); cloud_it++)
        {
            // end of a fragment
            if ((cloud_ptr->points[cloud_it].y > (curr_depth + max_scaled_fragment_depth))
                    || (cloud_ptr->points[cloud_it].y < (curr_depth - max_scaled_fragment_depth)) )
            {
                curr_depth = cloud_ptr->points[cloud_it].y;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                cloud_fragments.push_back(new_cloud);
            }

            // filling current cloud
            else
                (cloud_fragments.back())->points.push_back(cloud_ptr->points[cloud_it]);
        }

        return cloud_fragments;
    }

    else
        throw std::logic_error("cloud_manip::fragment_cloud : Invalid max depth.");
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_manip::crop_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr,
                       float x_thresh, float y_thresh, float z_thresh, float precision)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (unsigned int cloud_it = 0; cloud_it < cloud_ptr->points.size(); cloud_it++)
    {
        bool remove_point = false;

        if ((std::abs(cloud_ptr->points[cloud_it].x) > std::abs(x_thresh))
                && (!geom::aux::cmp_floats(x_thresh, 0, precision)))
            remove_point = true;

        if ((std::abs(cloud_ptr->points[cloud_it].y) > std::abs(y_thresh))
                && (!geom::aux::cmp_floats(y_thresh, 0, precision)))
            remove_point = true;

        if (std::abs(cloud_ptr->points[cloud_it].z) > std::abs(z_thresh)
                && (!geom::aux::cmp_floats(z_thresh, 0, precision)))
            remove_point = true;

        if (!remove_point)
            cropped_cloud_ptr->points.push_back(cloud_ptr->points[cloud_it]);
    }

    return cropped_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_manip::merge_clouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_fragments)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merge_result(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (unsigned int i = 0; i < cloud_fragments.size(); i++)
    {
        for (unsigned int j = 0; j < cloud_fragments[i]->points.size(); j++)
        {
            merge_result->points.push_back(cloud_fragments[i]->points[j]);
        }
    }

    return merge_result;
}

std::vector<point_xy_greyscale> cloud_manip::cloud_to_greyscale(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
    std::vector<point_xy_greyscale> gs_points;
    std::vector<float> z_coords = cloud_manip::cloud_z_coords(cloud_ptr);
    float z_min = *(std::min_element(z_coords.begin(), z_coords.end()));
    float z_max = *(std::max_element(z_coords.begin(), z_coords.end()));

    for (unsigned int cloud_it = 0; cloud_it < cloud_ptr->points.size(); cloud_it++)
    {
        point_xy_greyscale pt_xy_gs;
        float greyscale = geom::aux::map(cloud_ptr->points[cloud_it].z, z_min, z_max,
                                       0.0, 255.0);

        pt_xy_gs.x = cloud_ptr->points[cloud_it].x;
        pt_xy_gs.y = cloud_ptr->points[cloud_it].y;
        pt_xy_gs.greyscale((unsigned short)greyscale);
        gs_points.push_back(pt_xy_gs);
    }

    return gs_points;
}

void cloud_manip::cloud_homogenization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, short epsilon)
{
    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it = cloud_ptr->begin();
     cloud_it < cloud_ptr->end(); cloud_it++)
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

void cloud_manip::convertBoolToXYZRGB(pcl::PointCloud<clstr::point_clstr>::Ptr cloud_bool,
                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB)
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

void cloud_manip::convertXYZRGBToBool(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB,
                                      pcl::PointCloud<clstr::point_clstr>::Ptr cloud_bool)
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

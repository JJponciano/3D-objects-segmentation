#include "cloud_manip.h"

std::vector<float> cloud_object_segmentation::cloud_manip::cloud_x_coords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
    if (!cloud_ptr)
        throw cloud_object_segmentation::except::invalid_cloud_pointer();

    std::vector<float> x_coords;

    for (auto cloud_it = cloud_ptr->begin(); cloud_it < cloud_ptr->end(); cloud_it++)
        x_coords.push_back((float)(cloud_it->x));

    return x_coords;
}

std::vector<float> cloud_object_segmentation::cloud_manip::cloud_y_coords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
    if (!cloud_ptr)
        throw cloud_object_segmentation::except::invalid_cloud_pointer();

    std::vector<float> y_coords;

    for (auto cloud_it = cloud_ptr->begin(); cloud_it < cloud_ptr->end(); cloud_it++)
        y_coords.push_back((float)(cloud_it->y));

    return y_coords;
}

std::vector<float> cloud_object_segmentation::cloud_manip::cloud_z_coords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
    if (!cloud_ptr)
        throw cloud_object_segmentation::except::invalid_cloud_pointer();

    std::vector<float> z_coords;

    for (auto cloud_it = cloud_ptr->begin(); cloud_it < cloud_ptr->end(); cloud_it++)
        z_coords.push_back((float)(cloud_it->z));

    return z_coords;
}

void cloud_object_segmentation::cloud_manip::copy_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_ptr,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr dest_ptr)
{
    if (!src_ptr || !dest_ptr)
        throw std::invalid_argument("Invalid source or destination pointer.");

    for (auto cloud_it = src_ptr->begin(); cloud_it < src_ptr->end(); cloud_it++)
        dest_ptr->points.push_back(*cloud_it);
}

void cloud_object_segmentation::cloud_manip::scale_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, float x_scale, float y_scale,
                                 float z_scale)
{
    if (!cloud_ptr)
        throw cloud_object_segmentation::except::invalid_cloud_pointer();

    if (cloud_object_segmentation::aux::float_cmp(x_scale, 0.00, 0.005)
            || cloud_object_segmentation::aux::float_cmp(y_scale, 0.00, 0.005)
            || cloud_object_segmentation::aux::float_cmp(z_scale, 0.00, 0.005))
        throw std::invalid_argument("Scaling cloud by 0 will destroy the cloud.");

    for (auto cloud_it = cloud_ptr->points.begin(); cloud_it < cloud_ptr->points.end(); cloud_it++)
    {
        cloud_it->x *= x_scale;
        cloud_it->y *= y_scale;
        cloud_it->z *= z_scale;
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object_segmentation::cloud_manip::crop_cloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud_ptr, float x_thresh, float y_thresh,
        float z_thresh)
{
    if (!base_cloud_ptr)
        throw cloud_object_segmentation::except::invalid_cloud_pointer();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    bool crop_pt;

    for (auto cloud_it = base_cloud_ptr->begin(); cloud_it != base_cloud_ptr->end(); cloud_it++)
    {
        crop_pt = false;

        if ((std::abs(cloud_it->x) > std::abs(x_thresh)) && !cloud_object_segmentation::aux::float_cmp(x_thresh, 0.00, 0.005))
            crop_pt = true;

        if ((std::abs(cloud_it->y) > std::abs(y_thresh)) && !cloud_object_segmentation::aux::float_cmp(y_thresh, 0.00, 0.005))
            crop_pt = true;

        if ((std::abs(cloud_it->z) > std::abs(z_thresh)) && !cloud_object_segmentation::aux::float_cmp(z_thresh, 0.00, 0.005))
            crop_pt = true;

        if (!crop_pt)
            cropped_cloud_ptr->push_back(*cloud_it);
    }

    return cropped_cloud_ptr;
}

void cloud_object_segmentation::cloud_manip::homogenize_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, short epsilon)
{
    if (!cloud_ptr)
        throw cloud_object_segmentation::except::invalid_cloud_pointer();

    if (epsilon == 0)
        throw std::invalid_argument("Epsilon cannot be 0 for cloud homogenization.");

    for (auto cloud_it = cloud_ptr->begin(); cloud_it < cloud_ptr->end(); cloud_it++)
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

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_object_segmentation::cloud_manip::fragment_cloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, float max_scaled_fragment_depth)
{
    if (!cloud_ptr)
        throw cloud_object_segmentation::except::invalid_cloud_pointer();

    if ((cloud_object_segmentation::aux::float_cmp(max_scaled_fragment_depth, 0.00, 0.005)) || (max_scaled_fragment_depth < 0))
        throw std::invalid_argument("Invalid max fragment depth.");

    float curr_depth = FLT_MAX;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_fragments;

    for (auto cloud_it = cloud_ptr->begin(); cloud_it < cloud_ptr->end(); cloud_it++)
    {
        // end of a fragment
        if ((cloud_it->y > (curr_depth + max_scaled_fragment_depth))
                || (cloud_it->y < (curr_depth - max_scaled_fragment_depth)) )
        {
            curr_depth = cloud_it->y;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            cloud_fragments.push_back(new_cloud);
        }

        // filling current cloud
        else
            (cloud_fragments.back())->points.push_back(*cloud_it);
    }

    return cloud_fragments;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object_segmentation::cloud_manip::merge_clouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_fragments)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merge_result(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (auto fragm_it = cloud_fragments.begin(); fragm_it < cloud_fragments.end(); fragm_it++)
    {
        for (auto cloud_it = (*fragm_it)->begin(); cloud_it < (*fragm_it)->end(); cloud_it++)
            merge_result->points.push_back(*cloud_it);
    }

    return merge_result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object_segmentation::cloud_manip::cloud_to_rgb(pcl::PointCloud<pcl::PointXYZ>::Ptr white_cloud_ptr)
{
    if (!white_cloud_ptr)
        throw cloud_object_segmentation::except::invalid_cloud_pointer();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    uint8_t r = 255;
    uint8_t g = 255;
    uint8_t b = 255;   // make it white

    for (auto cloud_it = white_cloud_ptr->begin(); cloud_it < white_cloud_ptr->end(); cloud_it++)
    {
        pcl::PointXYZRGB rgb_point(r, g, b);
        rgb_point.x = cloud_it->x;
        rgb_point.y = cloud_it->y;
        rgb_point.z = cloud_it->z;

        rgb_cloud->push_back(rgb_point);
    }

    return rgb_cloud;
}

std::vector<cloud_object_segmentation::point_xy_greyscale> cloud_object_segmentation::cloud_manip::cloud_to_2d_greyscale(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
    if (!cloud_ptr)
        throw cloud_object_segmentation::except::invalid_cloud_pointer();

    std::vector<cloud_object_segmentation::point_xy_greyscale> greyscale_points;
    std::vector<float> z_coords = cloud_object_segmentation::cloud_manip::cloud_z_coords(cloud_ptr);
    float z_min = *(std::min_element(z_coords.begin(), z_coords.end()));
    float z_max = *(std::max_element(z_coords.begin(), z_coords.end()));

    for (auto cloud_it = cloud_ptr->begin(); cloud_it < cloud_ptr->end(); cloud_it++)
    {
        cloud_object_segmentation::point_xy_greyscale pt_xy_gs;
        float greyscale = cloud_object_segmentation::aux::map(cloud_it->z, z_min, z_max, 0.0, 255.0);

        pt_xy_gs.x = cloud_it->x;
        pt_xy_gs.y = cloud_it->y;
        pt_xy_gs.greyscale((unsigned short)greyscale);
        greyscale_points.push_back(pt_xy_gs);
    }

    return greyscale_points;
}

std::vector<cloud_object_segmentation::point_xy_rgb> cloud_to_2d_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
    if (!cloud_ptr)
        throw cloud_object_segmentation::except::invalid_cloud_pointer();

    std::vector<cloud_object_segmentation::point_xy_rgb> rgb_points;

    for (auto cloud_it = cloud_ptr->begin(); cloud_it < cloud_ptr->end(); cloud_it++)
    {
        cloud_object_segmentation::point_xy_rgb pt_xy_rgb;

        pt_xy_rgb.x = cloud_it->x;
        pt_xy_rgb.y = cloud_it->y;
        pt_xy_rgb.r(cloud_it->r);
        pt_xy_rgb.g(cloud_it->g);
        pt_xy_rgb.b(cloud_it->b);
        rgb_points.push_back(pt_xy_rgb);
    }

    return rgb_points;
}

std::vector<cloud_object_segmentation::point_xy_mixed> cloud_object_segmentation::cloud_manip::cloud_to_2d_mixed(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
    if (!cloud_ptr)
        throw cloud_object_segmentation::except::invalid_cloud_pointer();

    std::vector<cloud_object_segmentation::point_xy_mixed> mixed_points;
    std::vector<float> z_coords = cloud_object_segmentation::cloud_manip::cloud_z_coords(cloud_ptr);
    float z_min = *(std::min_element(z_coords.begin(), z_coords.end()));
    float z_max = *(std::max_element(z_coords.begin(), z_coords.end()));

    for (auto cloud_it = cloud_ptr->begin(); cloud_it < cloud_ptr->end(); cloud_it++)
    {
        cloud_object_segmentation::point_xy_mixed pt_xy_mixed;
        float greyscale = cloud_object_segmentation::aux::map(cloud_it->z, z_min, z_max, 0.0, 255.0);
        pt_xy_mixed.x = cloud_it->x;
        pt_xy_mixed.y = cloud_it->y;
        pt_xy_mixed.r(cloud_it->r);
        pt_xy_mixed.g(cloud_it->g);
        pt_xy_mixed.b(cloud_it->b);
        pt_xy_mixed.greyscale(greyscale);
        mixed_points.push_back(pt_xy_mixed);
    }

    return mixed_points;
}

void cloud_object_segmentation::cloud_manip::convertClstrToXYZRGB(pcl::PointCloud<clstr::point_clstr>::Ptr cloud_clstr,
                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB)
{
    cloud_RGB->width = cloud_clstr->width;
    cloud_RGB->height = cloud_clstr->height;
    cloud_RGB->resize(cloud_RGB->width * cloud_RGB->height);

    for(size_t i=0; i<cloud_clstr->points.size(); i++)
    {
        cloud_RGB->points[i].x = cloud_clstr->points[i].x;
        cloud_RGB->points[i].y = cloud_clstr->points[i].y;
        cloud_RGB->points[i].z = cloud_clstr->points[i].z;
        cloud_RGB->points[i].r = cloud_clstr->points[i].r;
        cloud_RGB->points[i].g = cloud_clstr->points[i].g;
        cloud_RGB->points[i].b = cloud_clstr->points[i].b;
    }
}

void cloud_object_segmentation::cloud_manip::convertXYZRGBToClstr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB,
                                      pcl::PointCloud<clstr::point_clstr>::Ptr cloud_clstr)
{
    cloud_clstr->width = cloud_RGB->width;
    cloud_clstr->height = cloud_RGB->height;
    cloud_clstr->resize(cloud_clstr->width * cloud_clstr->height);

    for(size_t i=0; i<cloud_RGB->points.size(); i++)
    {
        cloud_clstr->points[i].x = cloud_RGB->points[i].x;
        cloud_clstr->points[i].y = cloud_RGB->points[i].y;
        cloud_clstr->points[i].z = cloud_RGB->points[i].z;
        cloud_clstr->points[i].r = cloud_RGB->points[i].r;
        cloud_clstr->points[i].g = cloud_RGB->points[i].g;
        cloud_clstr->points[i].b = cloud_RGB->points[i].b;
    }
}

void cloud_object_segmentation::cloud_manip::giveRandomColorToCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    uint8_t r=rand()%255,g=rand()%255,b=rand()%255;

    for(auto cloud_it=cloud->begin(); cloud_it!=cloud->end(); cloud_it++)
    {
        (*cloud_it).r = r;
        (*cloud_it).g = g;
        (*cloud_it).b = b;
    }
}

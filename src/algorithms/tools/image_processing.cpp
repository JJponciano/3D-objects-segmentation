#include "image_processing.h"

namespace ns_cos = cloud_object_segmentation;

std::vector<float> ns_cos::image_processing::greyscale_vector_x_coords(
        std::vector<ns_cos::point_xy_greyscale> greyscale_vector)
{
    std::vector<float> x_coords;

    for (auto vector_it = greyscale_vector.begin(); vector_it < greyscale_vector.end(); vector_it++)
        x_coords.push_back((float)(vector_it->x));

    return x_coords;
}

std::vector<float> ns_cos::image_processing::greyscale_vector_y_coords(
        std::vector<ns_cos::point_xy_greyscale> greyscale_vector)
{
    std::vector<float> y_coords;

    for (auto vector_it = greyscale_vector.begin(); vector_it < greyscale_vector.end(); vector_it++)
        y_coords.push_back((float)(vector_it->y));

    return y_coords;
}

std::vector<float> ns_cos::image_processing::rgb_vector_x_coords(
        std::vector<ns_cos::point_xy_rgb> rgb_vector)
{
    std::vector<float> x_coords;

    for (auto vector_it = rgb_vector.begin(); vector_it < rgb_vector.end(); vector_it++)
        x_coords.push_back((float)(vector_it->x));

    return x_coords;
}

std::vector<float> ns_cos::image_processing::rgb_vector_y_coords(
        std::vector<ns_cos::point_xy_rgb> rgb_vector)
{
    std::vector<float> y_coords;

    for (auto vector_it = rgb_vector.begin(); vector_it < rgb_vector.end(); vector_it++)
        y_coords.push_back((float)(vector_it->y));

    return y_coords;
}

std::vector<float> ns_cos::image_processing::mixed_vector_x_coords(
        std::vector<ns_cos::point_xy_mixed> mixed_vector)
{
    std::vector<float> x_coords;

    for (auto vector_it = mixed_vector.begin(); vector_it < mixed_vector.end(); vector_it++)
        x_coords.push_back((float)(vector_it->x));

    return x_coords;
}

std::vector<float> ns_cos::image_processing::mixed_vector_y_coords(
        std::vector<ns_cos::point_xy_mixed> mixed_vector)
{
    std::vector<float> y_coords;

    for (auto vector_it = mixed_vector.begin(); vector_it < mixed_vector.end(); vector_it++)
        y_coords.push_back((float)(vector_it->y));

    return y_coords;
}

std::vector<unsigned short> ns_cos::image_processing::greyscale_image_values(
        ns_cos::image_greyscale gs_img)
{
    std::vector<unsigned short> greyscale_values;

    for (size_t y = 0; y < gs_img.height(); y++)
    {
        for (size_t x = 0; x < gs_img.width(); x++)
            greyscale_values.push_back(gs_img.get_grey_at(y, x));
    }

    return greyscale_values;
}

ns_cos::image_mixed ns_cos::image_processing::mixed_vector_to_image(
        std::vector<ns_cos::point_xy_mixed> mixed_vector, float x_epsilon)
{
    if (ns_cos::aux::float_cmp(x_epsilon, 0.000, 0.005) || x_epsilon < 0)
    {
        throw std::logic_error("x_epsilon must be bigger than 0.");
    }

    std::vector<float> x_coords = ns_cos::image_processing::mixed_vector_x_coords(mixed_vector);
    std::vector<float> y_coords = ns_cos::image_processing::mixed_vector_y_coords(mixed_vector);
    float x_min = *(std::min_element(x_coords.begin(), x_coords.end()));
    float y_min = *(std::min_element(y_coords.begin(), y_coords.end()));
    float y_max = *(std::max_element(y_coords.begin(), y_coords.end()));
    unsigned long width = 0;    // rgb image width
    unsigned long height = ((long)y_max - (long)y_min) + 1; // rgb image height

    // determining image width
    for (auto vector_it = mixed_vector.begin();
         vector_it < mixed_vector.end(); vector_it++)

    {
        unsigned long point_x_cell = (unsigned long)((vector_it->x - x_min) * x_epsilon * 10);

        if (point_x_cell > width)
            width = point_x_cell + 1;
    }

    ns_cos::image_mixed mixed_img(width, height);

    mixed_img.init();

    // writing data to image
    for (auto vector_it = mixed_vector.begin();
         vector_it < mixed_vector.end(); vector_it++)
    {
        unsigned long image_x = (unsigned long)((vector_it->x - x_min) * x_epsilon * 10);
        unsigned long image_y = vector_it->y - y_min;

        // avoiding std::out_of_range
        if (image_x >= mixed_img.width())
            image_x = mixed_img.width() - 1;

        // the point with the highest grey scale value colors the pixel
        if (mixed_img.get_grey_at(image_y, image_x) < vector_it->greyscale())
        {
            mixed_img.set_grey_at(image_y, image_x, vector_it->greyscale());
            mixed_img.set_rgb_at(image_y, image_x, (vector_it->rgb()));
        }
    }

    return mixed_img;
}

ns_cos::image_rgb ns_cos::image_processing::mixed_image_to_rgb(
        ns_cos::image_mixed mixed_img)
{
    ns_cos::image_rgb rgb_img(mixed_img.width(), mixed_img.height());

    for (size_t y = 0; y < mixed_img.height(); y++)
    {
        for (size_t x = 0; x < mixed_img.width(); x++)
            rgb_img.set_rgb_at(y, x, mixed_img.get_rgb_at(y, x));
    }

    return rgb_img;
}

ns_cos::image_greyscale ns_cos::image_processing::mixed_image_to_greyscale(
        ns_cos::image_mixed mixed_img)
{
    ns_cos::image_greyscale gs_img(mixed_img.width(), mixed_img.height());

    for (size_t y = 0; y < mixed_img.height(); y++)
    {
        for (size_t x = 0; x < mixed_img.width(); x++)
            gs_img.set_grey_at(y, x, mixed_img.get_grey_at(y, x));
    }

    return gs_img;
}

ns_cos::image_greyscale ns_cos::image_processing::cloud_to_depth_image(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, float x_epsilon)
{
    std::vector<ns_cos::point_xy_mixed> mixed_pt_arr;

    mixed_pt_arr = ns_cos::cloud_manip::cloud_to_2d_mixed(cloud_ptr);
    ns_cos::image_mixed mixed_img = ns_cos::image_processing::mixed_vector_to_image(mixed_pt_arr,
                                                                              x_epsilon);
    ns_cos::image_greyscale gs_img = ns_cos::image_processing::mixed_image_to_greyscale(mixed_img);

    return gs_img;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ns_cos::image_processing::mixed_image_to_cloud(
        ns_cos::image_mixed mixed_img,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud_ptr)
{
    if (!base_cloud_ptr)
        throw ns_cos::except::invalid_cloud_pointer();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr res_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<float> x_coords = ns_cos::cloud_manip::cloud_x_coords(base_cloud_ptr);
    std::vector<float> y_coords = ns_cos::cloud_manip::cloud_y_coords(base_cloud_ptr);
    std::vector<float> z_coords = ns_cos::cloud_manip::cloud_z_coords(base_cloud_ptr);

    // min and max coordinates for the map function
    float x_min = *(std::min_element(x_coords.begin(), x_coords.end()));
    float x_max = *(std::max_element(x_coords.begin(), x_coords.end()));
    float y_min = *(std::min_element(y_coords.begin(), y_coords.end()));
    float y_max = *(std::max_element(y_coords.begin(), y_coords.end()));
    float z_min = *(std::min_element(z_coords.begin(), z_coords.end()));
    float z_max = *(std::max_element(z_coords.begin(), z_coords.end()));

    for (size_t y = 0; y < mixed_img.height(); y++)
    {
        pcl::PointXYZRGB current_point;
        float cloud_y = ns_cos::aux::map(y, 0, mixed_img.height() - 1, y_min, y_max);

        for (size_t x = 0; x < mixed_img.width(); x++)
        {
            float cloud_x = ns_cos::aux::map(x, 0, mixed_img.width() - 1, x_min, x_max);
            float cloud_z = ns_cos::aux::map(mixed_img.get_grey_at(y, x), 0, 255, z_min, z_max);

            current_point.x = cloud_x;
            current_point.y = cloud_y;
            current_point.z = cloud_z;
            current_point.r = (float)mixed_img.get_red_at(y, x);
            current_point.g = (float)mixed_img.get_green_at(y, x);
            current_point.b = (float)mixed_img.get_blue_at(y, x);
            res_cloud_ptr->push_back(current_point);
        }
    }

    return res_cloud_ptr;
}

cv::Mat ns_cos::image_processing::greyscale_image_to_mat(ns_cos::image_greyscale gs_img)
{
    cv::Mat greyscale_mat(gs_img.height(), gs_img.width(), CV_8UC1);

    for (size_t y = 0; y < gs_img.height(); y++)
    {
        for (size_t x = 0; x < gs_img.width(); x++)
            greyscale_mat.at<uchar>(y, x) = gs_img.get_grey_at(y, x);
    }

    return greyscale_mat;
}

cv::Mat ns_cos::image_processing::rgb_image_to_mat(ns_cos::image_rgb rgb_img)
{
    cv::Mat rgb_mat(rgb_img.height(), rgb_img.width(), CV_8UC3);

    for (size_t y = 0; y < rgb_img.height(); y++)
    {
        for (size_t x = 0; x < rgb_img.width(); x++)
        {
            rgb_mat.at<cv::Vec3b>(y, x)[0] = rgb_img.get_red_at(y, x);
            rgb_mat.at<cv::Vec3b>(y, x)[1] = rgb_img.get_green_at(y, x);
            rgb_mat.at<cv::Vec3b>(y, x)[2] = rgb_img.get_blue_at(y, x);
        }
    }

    return rgb_mat;
}

ns_cos::image_greyscale ns_cos::image_processing::mat_to_greyscale_image(cv::Mat gs_mat)
{
    ns_cos::image_greyscale gs_img(gs_mat.cols, gs_mat.rows);

    gs_img.init();

    for (size_t y = 0; y < gs_img.height(); y++)
    {
        for (size_t x = 0; x < gs_img.width(); x++)
            gs_img.set_grey_at(y, x, (unsigned short)gs_mat.at<uchar>(y, x));
    }

    return gs_img;
}

ns_cos::image_rgb ns_cos::image_processing::mat_to_rgb_image(cv::Mat rgb_mat)
{
    /// TO-DO
    throw std::runtime_error("Not yet implemented.");
}

ns_cos::image_greyscale ns_cos::image_processing::detect_contours(
        ns_cos::image_greyscale gs_img, int hist_num_cls)
{
    cv::Mat gs_mat; // gs_img as a cv::Mat object
    int channels[] = {0};
    cv::Mat hist;   // histogram of gs_mat
    cv::Mat eq_hist;
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;
    cv::Mat grad;
    ns_cos::image_greyscale img_cont(gs_img.width(), gs_img.height());   // contours of the parameter image

    gs_mat = ns_cos::image_processing::greyscale_image_to_mat(gs_img);
    cv::calcHist(&gs_mat, 1, channels, cv::Mat(), hist, 1, &hist_num_cls, 0);
    cv::equalizeHist(gs_mat, eq_hist);
    cv::Sobel(gs_mat, grad_x, CV_64F, 1, 0, 3);
    cv::convertScaleAbs(grad_x, abs_grad_x);
    cv::Sobel(gs_mat, grad_y, CV_64F, 0, 1, 3);
    cv::convertScaleAbs(grad_y, abs_grad_y);
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
    img_cont = ns_cos::image_processing::mat_to_greyscale_image(grad);

    return img_cont;
}

void ns_cos::image_processing::remove_colors(ns_cos::image_mixed &mixed_img,
                                     std::vector<uint32_t> colors)
{
    for (size_t y = 0; y < mixed_img.height(); y++)
    {
        for (size_t x = 0; x < mixed_img.width(); x++)
        {
            for (auto color_it = colors.begin(); color_it < colors.end(); color_it++)
            {
                if (mixed_img.get_rgb_at(y, x) == (*color_it))
                    mixed_img.set_rgb_at(y, x, 0);
            }
        }
    }
}

void ns_cos::image_processing::normalize(ns_cos::image_greyscale *gs_img_ptr)
{
    if (!gs_img_ptr)
        throw std::invalid_argument("Invalid grey scale image pointer parameter.");

    std::vector<unsigned short> greyscale_values = ns_cos::image_processing::greyscale_image_values(*gs_img_ptr);
    unsigned short min_gs_val = *(std::min_element(greyscale_values.begin(), greyscale_values.end()));
    unsigned short max_gs_val = *(std::max_element(greyscale_values.begin(), greyscale_values.end()));

    for (size_t y = 0; y < gs_img_ptr->height(); y++)
    {
        for (size_t x = 0; x < gs_img_ptr->width(); x++)
        {
            unsigned short normalized_greyscale_val = (255 / (max_gs_val - min_gs_val))
                                                        * (gs_img_ptr->get_grey_at(y, x) - min_gs_val);

            gs_img_ptr->set_grey_at(y, x, normalized_greyscale_val);
        }
    }
}

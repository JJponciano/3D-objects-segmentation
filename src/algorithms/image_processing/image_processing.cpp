#include "image_processing.h"

std::vector<float> image_processing::greyscale_x_coords(std::vector<point_xy_greyscale> greyscale_vector)
{
    std::vector<float> x_coords;

    for (std::vector<point_xy_greyscale>::iterator vector_it = greyscale_vector.begin();
         vector_it < greyscale_vector.end(); vector_it++)
        x_coords.push_back((float)(vector_it->x));

    return x_coords;
}

std::vector<float> image_processing::greyscale_y_coords(std::vector<point_xy_greyscale> greyscale_vector)
{
    std::vector<float> y_coords;

    for (std::vector<point_xy_greyscale>::iterator vector_it = greyscale_vector.begin();
         vector_it < greyscale_vector.end(); vector_it++)
        y_coords.push_back((float)(vector_it->y));

    return y_coords;
}

std::vector<unsigned short> image_processing::greyscale_values(greyscale_image gs_img)
{
    std::vector<unsigned short> greyscale_values;

    for (unsigned int y = 0; y < gs_img.height(); y++)
    {
        for (unsigned int x = 0; x < gs_img.width(); x++)
        {
            greyscale_values.push_back(gs_img.get_grey_at(y, x));
        }
    }

    return greyscale_values;
}

greyscale_image image_processing::greyscale_to_image(std::vector<point_xy_greyscale> greyscale_vector, float x_epsilon)
{
    std::vector<float> x_coords = image_processing::greyscale_x_coords(greyscale_vector);
    std::vector<float> y_coords = image_processing::greyscale_y_coords(greyscale_vector);
    float x_min = *(std::min_element(x_coords.begin(), x_coords.end()));   // smallest x coordinate
    float y_min = *(std::min_element(y_coords.begin(), y_coords.end()));   // smallest y coordinate
    float y_max = *(std::max_element(y_coords.begin(), y_coords.end()));   // biggest y coordinate
    unsigned long width = 0;    // grey scale image width
    unsigned long height = ((long)y_max - (long)y_min) + 1;   // grey scale image height

    // determining image width
    for (std::vector<point_xy_greyscale>::iterator vector_it = greyscale_vector.begin();
         vector_it < greyscale_vector.end(); vector_it++)
    {
        unsigned long point_x_cell = (unsigned long)((vector_it->x - x_min) * x_epsilon * 10);

        if (point_x_cell > width)
            width = point_x_cell + 1;
    }

    greyscale_image gs_img(width, height);

    // initializing image
    for (unsigned long i = 0; i < gs_img.height(); i++)
    {
        for (unsigned long j = 0; j < gs_img.width(); j++)
        {
            gs_img.set_grey_at(i, j, 0);
        }
    }

    // filling image
    for (std::vector<point_xy_greyscale>::iterator vector_it = greyscale_vector.begin();
         vector_it < greyscale_vector.end(); vector_it++)
    {
        unsigned long image_x = (unsigned long)((vector_it->x - x_min) * x_epsilon * 10);
        unsigned long image_y = vector_it->y - y_min;

        if (gs_img.get_grey_at(image_y, image_x) < vector_it->greyscale())
            gs_img.set_grey_at(image_y, image_x, vector_it->greyscale());
    }

    return gs_img;
}

void image_processing::normalize(greyscale_image *gs_img_ptr)
{
    std::vector<unsigned short> greyscale_values = image_processing::greyscale_values(*gs_img_ptr);
    unsigned short min_gs_val = *(std::min_element(greyscale_values.begin(), greyscale_values.end()));
    unsigned short max_gs_val = *(std::max_element(greyscale_values.begin(), greyscale_values.end()));

    for (unsigned long y = 0; y < gs_img_ptr->height(); y++)
    {
        for (unsigned long x = 0; x < gs_img_ptr->width(); x++)
        {
            unsigned short normalized_greyscale_val = (255 / (max_gs_val - min_gs_val))
                                                        * (gs_img_ptr->get_grey_at(y, x) - min_gs_val);

            gs_img_ptr->set_grey_at(y, x, normalized_greyscale_val);
        }
    }
}

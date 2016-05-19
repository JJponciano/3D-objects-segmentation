#ifndef IMAGE_IO_H
#define IMAGE_IO_H

#include "../2d/image.h"
#include "../2d/image_greyscale.h"
#include "../2d/image_rgb.h"
#include "../2d/point_xy_greyscale.h"

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <ios>
#include <exception>
#include <type_traits>

#include <boost/lexical_cast.hpp>

namespace image_io
{
    /**
     * @brief export_greyscale_vector exports a greyscale vector to a text file
     * @param path is a string representing a unique location in the file system
     * @param greyscale_vector is the point_xy_greyscale array to be exported
     */
    void export_greyscale_vector(std::string path, std::vector<point_xy_greyscale> greyscale_vector);

    /**
     * @brief export_image exports a greyscale image to a .pgm file
     * @param path is a string representing a unique location in the file system
     * @param max_grey_value is the maximum value of grey in the image
     * @param gs_img is the image to be exported
     */
    void export_greyscale_image(std::string path, unsigned short max_grey_value, image_greyscale gs_img);

    /**
     * @brief export_image exports a color image to a .pgm file
     * @param path is a string representing a unique location in the file system
     * @param max_rgb_value is the maximum value of red, green and blue in the image
     * @param rgb_img is the image to be exported
     */
    void export_rgb_image(std::string path, unsigned int max_rgb_value, image_rgb rgb_img);
}

#endif

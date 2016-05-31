/**
  @author Vlad-Adrian Moglan
  @brief contains image in and out functions
  */

#ifndef IMAGE_IO_H
#define IMAGE_IO_H

#include "../2d/image_greyscale.h"
#include "../2d/image_rgb.h"
#include "../2d/point_xy_greyscale.h"
#include "../except/invalid_path.h"

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <ios>
#include <exception>
#include <type_traits>

#include <stdlib.h>
#include <stdio.h>

#include <boost/lexical_cast.hpp>

namespace image_io
{
    /**
     * @brief import_greyscale_image creates an image_greyscale object from a pgm file
     * @param path is a string representing a unique location in the file system
     * @throw invalid_path if failed to open file
     * @return the imported grey scale image
     */
    image_greyscale import_greyscale_image(std::string path);

    /**
     * @brief export_image exports a greyscale image to a .pgm file
     * @param path is a string representing a unique location in the file system
     * @param max_grey_value is the maximum value of grey in the image
     * @param gs_img is the image to be exported
     * @throw invalid_path if failed to open file
     * @throw std::logic_error if max_grey_value is bigger than 255 or if rgb_img has size 0
     */
    void export_greyscale_image(std::string path, unsigned short max_grey_value, image_greyscale gs_img);

    /**
     * @brief import_rgb_image creates an image_rgb object from a ppm file
     * @param path is a string representing a unique location in the file system
     * @throw invalid_path if failed to open file
     * @return the imported rgb image
     */
    image_rgb import_rgb_image(std::string path);

    /**
     * @brief export_image exports a color image to a .ppm file
     * @param path is a string representing a unique location in the file system
     * @param max_rgb_value is the maximum value of red, green and blue in the image
     * @param rgb_img is the image to be exported
     * @throw invalid_path if failed to open file
     * @throw std::logic_error if max_rgb_value is bigger than 255 or if rgb_img has size 0
     */
    void export_rgb_image(std::string path, unsigned int max_rgb_value, image_rgb rgb_img);
}

#endif

#include "greyscale_image.h"

greyscale_image::greyscale_image(int width, int height) : image(width, height)
{
}

unsigned short greyscale_image::get_grey_at(int y, int x)
{
    try
    {
        return this->_pixels[y][x];
    }

    catch (std::out_of_range err)
    {
        std::string err_msg = err.what();

        throw "greyscale_image::get_grey_at : " + err_msg;
    }
}

void greyscale_image::set_grey_at(int y, int x, unsigned short grey)
{
    try
    {
        this->_pixels[y][x] = grey;
    }

    catch (std::out_of_range err)
    {
        std::string err_msg = err.what();

        throw "greyscale_image::get_grey_at : " + err_msg;
    }
}

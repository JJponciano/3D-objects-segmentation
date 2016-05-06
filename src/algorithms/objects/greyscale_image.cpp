#include "greyscale_image.h"

greyscale_image::greyscale_image(unsigned long width, unsigned long height) : image(width, height)
{
    _pixels.resize(height);

    for (unsigned int i = 0; i < _pixels.size(); i++)
    {
        _pixels[i].resize(width);
    }
}

unsigned short greyscale_image::get_grey_at(unsigned long y, unsigned long x)
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

void greyscale_image::set_grey_at(unsigned long y, unsigned long x, unsigned short grey)
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

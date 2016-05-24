#include "image_greyscale.h"

image_greyscale::image_greyscale(unsigned long width, unsigned long height) : image(width, height)
{
    _pixels.resize(height);

    for (unsigned long i = 0; i < _pixels.size(); i++)
    {
        _pixels[i].resize(width);
    }
}

unsigned short image_greyscale::get_grey_at(unsigned long y, unsigned long x) const    
{
    return this->_pixels[y][x];
}

void image_greyscale::set_grey_at(unsigned long y, unsigned long x, unsigned short grey)
{
    this->_pixels[y][x] = grey;
}

void image_greyscale::init()
{
    for (unsigned long y = 0; y < this->height(); y++)
    {
        for (unsigned long x = 0; x < this->width(); x++)
        {
            this->set_grey_at(y, x, 0);
        }
    }
}

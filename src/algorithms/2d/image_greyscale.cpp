#include "image_greyscale.h"

cloud_object_segmentation::image_greyscale::image_greyscale(size_t width, size_t height) : image(width, height)
{
    _pixels.resize(height);

    for (size_t i = 0; i < _pixels.size(); i++)
        _pixels[i].resize(width);
}

unsigned short cloud_object_segmentation::image_greyscale::get_grey_at(size_t y, size_t x) const
{
    return this->_pixels[y][x];
}

void cloud_object_segmentation::image_greyscale::set_grey_at(size_t y, size_t x, unsigned short grey)
{
    this->_pixels[y][x] = grey;
}

void cloud_object_segmentation::image_greyscale::init()
{
    for (size_t y = 0; y < this->height(); y++)
    {
        for (size_t x = 0; x < this->width(); x++)
            this->set_grey_at(y, x, 0);
    }
}

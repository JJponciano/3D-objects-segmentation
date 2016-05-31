#include "image_mixed.h"

image_mixed::image_mixed(size_t width, size_t height) : image(width, height)
{
    _pixels.resize(height);

    for (size_t i = 0; i < _pixels.size(); i++)
        _pixels[i].resize(width);

    for (size_t y = 0; y < this->height(); y++)
    {
        for (size_t x = 0; x < this->width(); x++)
            _pixels[y][x] = new std::pair<unsigned short, uint32_t>;
    }
}

unsigned short image_mixed::get_grey_at(size_t y, size_t x) const
{
    return this->_pixels[y][x]->first;
}

uint32_t image_mixed::get_rgb_at(size_t y, size_t x) const
{
    return this->_pixels[y][x]->second;
}

uint8_t image_mixed::get_red_at(size_t y, size_t x) const
{
   return (this->_pixels[y][x]->second >> 16) & 0x0000ff;
}

uint8_t image_mixed::get_green_at(size_t y, size_t x) const
{
    return (this->_pixels[y][x]->second >> 8) & 0x0000ff;
}

uint8_t image_mixed::get_blue_at(size_t y, size_t x) const
{
    return (this->_pixels[y][x]->second) & 0x0000ff;
}

void image_mixed::set_grey_at(size_t y, size_t x, unsigned short grey)
{
    this->_pixels[y][x]->first = grey;
}

void image_mixed::set_rgb_at(size_t y, size_t x, uint32_t rgb)
{
    this->_pixels[y][x]->second = rgb;
}

void image_mixed::init()
{
    for (size_t y = 0; y < this->height(); y++)
    {
        for (size_t x = 0; x < this->width(); x++)
        {
            this->_pixels[y][x]->first = 0;
            this->_pixels[y][x]->second = (uint32_t)0;
        }
    }
}

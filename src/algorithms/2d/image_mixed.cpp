#include "image_mixed.h"

image_mixed::image_mixed(unsigned long width, unsigned long height) : image(width, height)
{
    _pixels.resize(height);

    for (unsigned long i = 0; i < _pixels.size(); i++)
    {
        _pixels[i].resize(width);
    }

    for (unsigned long y = 0; y < this->height(); y++)
    {
        for (unsigned long x = 0; x < this->width(); x++)
        {
            _pixels[y][x] = new std::pair<unsigned short, uint32_t>;
        }
    }
}

unsigned short image_mixed::get_grey_at(unsigned long y, unsigned long x) const    
{
    return this->_pixels[y][x]->first;
}

void image_mixed::set_grey_at(unsigned long y, unsigned long x, unsigned short grey)    
{
    this->_pixels[y][x]->first = grey;
}

uint32_t image_mixed::get_rgb_at(unsigned long y, unsigned long x) const    
{
    return this->_pixels[y][x]->second;
}

void image_mixed::set_rgb_at(unsigned long y, unsigned long x, uint32_t rgb)    
{
    this->_pixels[y][x]->second = rgb;
}

uint8_t image_mixed::get_red_at(unsigned long y, unsigned long x) const   
{
   return (this->_pixels[y][x]->second >> 16) & 0x0000ff;
}

uint8_t image_mixed::get_green_at(unsigned long y, unsigned long x) const    
{
    return (this->_pixels[y][x]->second >> 8) & 0x0000ff;
}

uint8_t image_mixed::get_blue_at(unsigned long y, unsigned long x) const
{
    return (this->_pixels[y][x]->second) & 0x0000ff;
}

void image_mixed::init()
{
    for (unsigned long y = 0; y < this->height(); y++)
    {
        for (unsigned long x = 0; x < this->width(); x++)
        {
            this->_pixels[y][x]->first = 0;
            this->_pixels[y][x]->second = (uint32_t)0;
        }
    }
}

#include "image_rgb.h"

image_rgb::image_rgb(unsigned long width, unsigned long height) : image(width, height)
{
    this->_pixels.resize(height);

    for (unsigned long i = 0; i < this->_pixels.size(); i++)
    {
        this->_pixels[i].resize(width);
    }
}

uint32_t image_rgb::get_rgb_at(unsigned long y, unsigned long x) const    
{
    try
    {
        return this->_pixels[y][x];
    }

    catch (const std::out_of_range& oor)
    {
        throw oor;
    }
}

void image_rgb::set_rgb_at(unsigned long y, unsigned long x, uint32_t rgb)    
{
    try
    {
        this->_pixels[y][x] = rgb;
    }

    catch (const std::out_of_range& oor)
    {
        throw oor;
    }
}

uint8_t image_rgb::get_red_at(unsigned long y, unsigned long x) const    
{
    try
    {
        return (this->_pixels[y][x] >> 16) & 0x0000ff;
    }

    catch (const std::out_of_range& oor)
    {
        throw oor;
    }
}

uint8_t image_rgb::get_green_at(unsigned long y, unsigned long x) const   
{
    try
    {
        return (this->_pixels[y][x] >> 8) & 0x0000ff;
    }

    catch (const std::out_of_range& oor)
    {
        throw oor;
    }
}

uint8_t image_rgb::get_blue_at(unsigned long y, unsigned long x) const    
{
    try
    {
        return (this->_pixels[y][x]) & 0x0000ff;
    }

    catch (const std::out_of_range& oor)
    {
        throw oor;
    }
}

void image_rgb::init()
{
    for (unsigned long y = 0; y < this->height(); y++)
    {
        for (unsigned long x = 0; x < this->width(); x++)
            this->_pixels[y][x] = 0;
    }
}
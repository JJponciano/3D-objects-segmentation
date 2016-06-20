#include "image_rgb.h"

namespace ns_cos = cloud_object_segmentation;

ns_cos::image_rgb::image_rgb(size_t width, size_t height) : image(width, height)
{
    this->_pixels.resize(height);

    for (size_t i = 0; i < this->_pixels.size(); i++)
        this->_pixels[i].resize(width);
}

uint32_t ns_cos::image_rgb::get_rgb_at(size_t y, size_t x) const
{
    return this->_pixels[y][x];
}

void ns_cos::image_rgb::set_rgb_at(size_t y, size_t x, uint32_t rgb)
{
    this->_pixels[y][x] = rgb;
}

uint8_t ns_cos::image_rgb::get_red_at(size_t y, size_t x) const
{
    return (this->_pixels[y][x] >> 16) & 0x0000ff;
}

uint8_t ns_cos::image_rgb::get_green_at(size_t y, size_t x) const
{
    return (this->_pixels[y][x] >> 8) & 0x0000ff;
}

uint8_t ns_cos::image_rgb::get_blue_at(size_t y, size_t x) const
{
    return (this->_pixels[y][x]) & 0x0000ff;
}

void ns_cos::image_rgb::init()
{
    for (size_t y = 0; y < this->height(); y++)
    {
        for (size_t x = 0; x < this->width(); x++)
            this->_pixels[y][x] = 0;
    }
}

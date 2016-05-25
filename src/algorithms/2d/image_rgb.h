#ifndef IMAGE_RGB_H
#define IMAGE_RGB_H

#include "image.h"

#include <stdint.h>
#include <stdexcept>
#include <vector>

class image_rgb : public image
{
private:
    std::vector<std::vector<uint32_t>> _pixels;
public:
    /**
     * @brief image_rgb is the class constructor
     * @param width is the width of the image in pixels
     * @param height is the height of the image in pixels
     */
    image_rgb(size_t width, size_t height);

    /** @brief get_rgb_at gets rgb value at coordinates [y, x] */
    uint32_t get_rgb_at(size_t y, size_t x) const;

    /** @brief get_red_at gets red value at coordinates [y, x] */
    uint8_t get_red_at(size_t y, size_t x) const;

    /** @brief get_green_at gets green value at coordinates [y, x] */
    uint8_t get_green_at(size_t y, size_t x) const;

    /** @brief get_blue_at gets blue value at coordinates [y, x] */
    uint8_t get_blue_at(size_t y, size_t x) const;

    /** @brief set_rgb_at sets rgb value at coordinates [y, x] */
    void set_rgb_at(size_t y, size_t x, uint32_t rgb);

    /** @brief init initializes image pixels */
    virtual void init();
};

#endif // IMAGE_RGB_H

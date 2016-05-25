#ifndef IMAGE_MIXED_H
#define IMAGE_MIXED_H

#include "image.h"

#include <stdint.h>
#include <stdexcept>
#include <vector>

class image_mixed : public image
{
private:
    std::vector<std::vector<std::pair<unsigned short, uint32_t> *>> _pixels;
public:
    /**
     * @brief image_mixed is the class constructor
     * @param width is the width of the image in pixels
     * @param height is the height of the image in pixels
     */
    image_mixed(size_t width, size_t height);

    /** @brief get_grey_at gets grey value at coordinates [y, x] */
    unsigned short get_grey_at(size_t y, size_t x) const;

    /** @brief get_rgb_at gets rgb value at coordinates [y, x] */
    uint32_t get_rgb_at(size_t y, size_t x) const;

    /** @brief get_red_at gets red value at coordinates [y, x] */
    uint8_t get_red_at(size_t y, size_t x) const;

    /** @brief get_green_at gets green value at coordinates [y, x] */
    uint8_t get_green_at(size_t y, size_t x) const;

    /** @brief get_blue_at gets blue value at coordinates [y, x] */
    uint8_t get_blue_at(size_t y, size_t x) const;

    /** @brief set_grey_at sets grey value at coordinates [y, x] */
    void set_grey_at(size_t y, size_t x, unsigned short grey);

    /** @brief set_rgb_at sets rgb value at coordinates [y, x] */
    void set_rgb_at(size_t y, size_t x, uint32_t rgb);

    /** @brief init initializes image pixels */
    virtual void init();
};

#endif // IMAGE_MIXED_H

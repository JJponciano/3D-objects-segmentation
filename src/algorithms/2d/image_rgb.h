#ifndef IMAGE_RGB_H
#define IMAGE_RGB_H

#include "image.h"

#include <stdint.h>
#include <vector>
#include <stdexcept>

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
    image_rgb(unsigned long width, unsigned long height);

    /** @brief getters and setters */
    uint32_t get_rgb_at(unsigned long y, unsigned long x) const;
    void set_rgb_at(unsigned long y, unsigned long x, uint32_t rgb);
    uint8_t get_red_at(unsigned long y, unsigned long x) const;
    uint8_t get_green_at(unsigned long y, unsigned long x) const;
    uint8_t get_blue_at(unsigned long y, unsigned long x) const;

    /** @brief initializes image pixels */
    virtual void init();
};

#endif // IMAGE_RGB_H

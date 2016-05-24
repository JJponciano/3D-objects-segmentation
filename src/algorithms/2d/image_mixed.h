#ifndef IMAGE_MIXED_H
#define IMAGE_MIXED_H

#include "image.h"

#include <stdint.h>
#include <vector>
#include <stdexcept>

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
    image_mixed(unsigned long width, unsigned long height);

    // getters and setters
    unsigned short get_grey_at(unsigned long y, unsigned long x) const;
    void set_grey_at(unsigned long y, unsigned long x, unsigned short grey);
    uint32_t get_rgb_at(unsigned long y, unsigned long x) const;
    void set_rgb_at(unsigned long y, unsigned long x, uint32_t rgb);
    uint8_t get_red_at(unsigned long y, unsigned long x) const;
    uint8_t get_green_at(unsigned long y, unsigned long x) const;
    uint8_t get_blue_at(unsigned long y, unsigned long x) const;

    /** @brief initializes image pixels */
    virtual void init();
};

#endif // IMAGE_MIXED_H

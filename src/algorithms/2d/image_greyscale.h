#ifndef GREYSCALE_IMAGE_H
#define GREYSCALE_IMAGE_H

#include "image.h"

#include <vector>
#include <stdexcept>

class image_greyscale : public image
{
private:
    std::vector<std::vector<unsigned short>> _pixels;
public:
    /**
     * @brief image_greyscale is the class constructor
     * @param width is the width of the image in pixels
     * @param height is the height of the image in pixels
     */
    image_greyscale(unsigned long width, unsigned long height);

    // getters and setters
    unsigned short get_grey_at(unsigned long y, unsigned long x) const;
    void set_grey_at(unsigned long y, unsigned long x, unsigned short grey);

    /** @brief initializes image pixels */
    virtual void init();
};

#endif // GREYSCALE_IMAGE_H

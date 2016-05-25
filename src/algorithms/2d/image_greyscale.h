#ifndef IMAGE_GREYSCALE_H
#define IMAGE_GREYSCALE_H

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
    image_greyscale(size_t width, size_t height);

    /** @brief get_grey_at gets grey value at coordinates [y, x] */
    unsigned short get_grey_at(size_t y, size_t x) const;

    /** @brief set_grey_at sets grey value at coordinates [y, x] */
    void set_grey_at(size_t y, size_t x, unsigned short grey);

    /** @brief init initializes image pixels */
    virtual void init();
};

#endif // IMAGE_GREYSCALE_H

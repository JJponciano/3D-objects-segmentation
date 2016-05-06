#ifndef GREYSCALE_IMAGE_H
#define GREYSCALE_IMAGE_H

#include "image.h"

#include <vector>
#include <stdexcept>

class greyscale_image : public image
{
private:
    std::vector<std::vector<unsigned short>> _pixels;
public:
    greyscale_image(unsigned long width, unsigned long height);
    unsigned short get_grey_at(unsigned long y, unsigned long x);
    void set_grey_at(unsigned long y, unsigned long x, unsigned short grey);
};

#endif // GREYSCALE_IMAGE_H

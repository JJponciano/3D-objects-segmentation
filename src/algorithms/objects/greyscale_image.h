#ifndef GREYSCALE_IMAGE_H
#define GREYSCALE_IMAGE_H

#include <vector>

class greyscale_image
{
private:
    int _width;
    int _height;
    std::vector<std::vector<short>> _pixels;
public:
    greyscale_image();
    greyscale_image(int width, int height);
    int width() { return _width; }
    int height() { return _height; }
    int resolution() { return _width * _height; }
};

#endif // GREYSCALE_IMAGE_H

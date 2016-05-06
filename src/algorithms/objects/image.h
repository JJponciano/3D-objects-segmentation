#ifndef IMAGE_H
#define IMAGE_H

class image
{
private:
    unsigned long _width;
    unsigned long _height;
public:
    image(unsigned long width, unsigned long height);
    unsigned long width() { return _width; }
    unsigned long height() { return _height; }
    unsigned long resolution() { return _width * _height; }
};

#endif // IMAGE_H

#ifndef IMAGE_H
#define IMAGE_H

class image
{
private:
    int _width;
    int _height;
public:
    image(int width, int height);
    int width() { return _width; }
    int height() { return _height; }
    int resolution() { return _width * _height; }
};

#endif // IMAGE_H

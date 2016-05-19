#ifndef IMAGE_H
#define IMAGE_H

class image
{
private:
    unsigned long _width;
    unsigned long _height;
public:
    /**
     * @brief image is the constructor
     * @param width is the width of the image in pixels
     * @param height is the height of the image in pixels
     */
    image(unsigned long width, unsigned long height);

    /** @brief getters and setters */
    unsigned long width() const { return _width; }
    unsigned long height() const { return _height; }
    unsigned long resolution() const { return _width * _height; }

    /** @brief initializes the image pixels */
    virtual void init() = 0;
};

#endif // IMAGE_H

/**
  * @author Vlad-Adrian Moglan
  */

#ifndef IMAGE_H
#define IMAGE_H

#include <stdlib.h>

namespace cloud_object_segmentation
{
    /**
     * @brief The image class is an abstract class representing a basic image
     */
    class image
    {
    private:
        size_t _width;
        size_t _height;
    public:
        /**
         * @brief image is the constructor
         * @param width is the width of the image in pixels
         * @param height is the height of the image in pixels
         */
        image(size_t width, size_t height);

        /** @brief width gets width */
        size_t width() const { return _width; }

        /** @brief height gets height */
        size_t height() const { return _height; }

        /** @brief resolution return _width * _height */
        size_t resolution() const { return _width * _height; }

        /** @brief init initializes the image pixels */
        virtual void init() = 0;
    };
}

#endif // IMAGE_H

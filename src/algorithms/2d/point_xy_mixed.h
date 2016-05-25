#ifndef POINT_XY_MIXED_H
#define POINT_XY_MIXED_H

#include "point_xy_greyscale.h"
#include "point_xy_rgb.h"

#include <stdint.h>

#include <pcl/point_types.h>

class point_xy_mixed : public point_xy_rgb
{
private:
    unsigned short _greyscale;  // greyscale value
public:
    /**
     * @brief point_xy_mixed is the default constructor
     */
    point_xy_mixed();

    /**
     * @brief point_xy_mixed is the constructor that initializes the rgb and grey scale values of the point
     * @param rgb is the value used for initializing _rgb
     * @param greyscale is the value used for initializing _greyscale
     */
    point_xy_mixed(uint32_t rgb, unsigned short greyscale);

    /** @brief greyscale gets point grey scale value */
    unsigned short greyscale() const { return _greyscale; }

    /** @brief greyscale sets point greyscale value */
    void greyscale(unsigned short greyscale) { this->_greyscale = greyscale; }
};

#endif // POINT_XY_MIXED_H

#ifndef POINT_XY_GREYSCALE_H
#define POINT_XY_GREYSCALE_H

#include <pcl/point_types.h>

class point_xy_greyscale : public pcl::PointXY
{
private:
    unsigned short _greyscale;  // grey scale value; between 0 and 255
public:
    /** @brief is the default constructor */
    point_xy_greyscale();

    /**
     * @brief point_xy_greyscale is a constructor initializing greyscale
     * @param greyscale is the grey scale value for initializing the point
     */
    point_xy_greyscale(unsigned short greyscale);

    /** @brief greyscale gets point grey scale value */
    unsigned short greyscale() const { return _greyscale; }

    /** @brief greyscale sets point grey scale value */
    void greyscale(unsigned short greyscale) { this->_greyscale = greyscale; }
};

#endif // POINT_XY_GREYSCALE_H

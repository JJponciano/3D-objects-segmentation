#ifndef POINT_XY_GREYSCALE_H
#define POINT_XY_GREYSCALE_H

#include <pcl/point_types.h>

class point_xy_greyscale : public pcl::PointXY
{
private:
    unsigned short _greyscale;
public:
    point_xy_greyscale();
    point_xy_greyscale(unsigned short greyscale);
    unsigned short greyscale() { return _greyscale; }
    void greyscale(unsigned short greyscale) { this->_greyscale = greyscale; }
};

#endif // POINT_XY_GREYSCALE_H

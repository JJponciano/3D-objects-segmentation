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

    /** @brief getters and setters */
    unsigned short greyscale() const { return _greyscale; }
    void greyscale(unsigned short greyscale) { this->_greyscale = greyscale; }
    /*
    uint32_t rgb() const { return _rgb; }
    void rgb(uint32_t rgb) { this->_rgb = rgb; }
    uint8_t r() const { return (rgb() >> 16) & 0x0000ff; }
    void r(uint8_t r) { _rgb = ((uint32_t)r << 16) | (uint32_t)g() << 8 | (uint32_t)b(); }
    uint8_t g() const { return (rgb() >> 8) & 0x0000ff; }
    void g(uint8_t g) { _rgb = ((uint32_t)r() << 16) | (uint32_t)g << 8 | (uint32_t)b(); }
    uint8_t b() const { return (rgb() & 0x0000ff); }
    void b(uint8_t b) { _rgb = ((uint32_t)r() << 16) | (uint32_t)g() << 8 | (uint32_t)b; }*/
};

#endif // POINT_XY_MIXED_H

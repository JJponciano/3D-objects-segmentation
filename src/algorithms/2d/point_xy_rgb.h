#ifndef POINT_XY_RGB_H
#define POINT_XY_RGB_H

#include <stdint.h>

#include <pcl/point_types.h>

class point_xy_rgb : public pcl::PointXY
{
private:
    uint32_t _rgb;  // red, green and blue values concatenated
public:
    /**
     * @brief point_xy_rgb is the default constructor
     */
    point_xy_rgb();

    /**
     * @brief point_xy_rgb is the constructor that initializes _rgb
     * @param rgb is the value used for initializing _rgb
     */
    point_xy_rgb(uint32_t rgb);

    // getters and setters
    uint32_t rgb() const { return _rgb; }
    void rgb(uint32_t rgb) { this->_rgb = rgb; }
    uint8_t r() const { return (rgb() >> 16) & 0x0000ff; }
    void r(uint8_t r) { _rgb = ((uint32_t)r << 16) | (uint32_t)g() << 8 | (uint32_t)b(); }
    uint8_t g() const { return (rgb() >> 8) & 0x0000ff; }
    void g(uint8_t g) { _rgb = ((uint32_t)r() << 16) | (uint32_t)g << 8 | (uint32_t)b(); }
    uint8_t b() const { return (rgb() & 0x0000ff); }
    void b(uint8_t b) { _rgb = ((uint32_t)r() << 16) | (uint32_t)g() << 8 | (uint32_t)b; }
};

#endif // POINT_XY_RGB_H

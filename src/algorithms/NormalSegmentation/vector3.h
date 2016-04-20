#ifndef VECTOR3_H
#define VECTOR3_H
#define _USE_MATH_DEFINES

#include <cmath>

namespace geom
{
    namespace vectors
    {
        class vector3
        {
        private:
            float x;
            float y;
            float z;
        public:
            vector3();
            vector3(float x, float y, float z);

            // getters
            float get_x() { return x; }
            float get_y() { return y; }
            float get_z() { return z; }
            float get_magn() { return std::sqrt(std::pow(this->get_x(), 2) + std::pow(this->get_y(), 2) + std::pow(this->get_z(), 2)); }

            // setters
            void set_x(float x) { this->x = x; }
            void set_y(float y) { this->y = y; }
            void set_z(float z) { this->z = z; }

        };
    }
}

#endif // VECTOR3_H

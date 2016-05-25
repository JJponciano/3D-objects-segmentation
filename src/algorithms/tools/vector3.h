/**
  * @brief a class representing the blueprints for vector3
  * @author Vlad-Adrian Moglan
  */

#ifndef VECTOR3_H
#define VECTOR3_H
#define _USE_MATH_DEFINES

#include <cmath>

namespace aux
{
    class vector3
    {
    private:
        float _x;
        float _y;
        float _z;
    public:
        vector3();
        vector3(float x, float y, float z);

        /// getters
        float x() const { return _x; }

        float y() const { return _y; }

        float z() const { return _z; }

        float magnitude() const { return std::sqrt(std::pow(this->x(), 2) + std::pow(this->y(), 2) + std::pow(this->z(), 2)); }

        /// setters
        void x(float x) { _x = x; }

        void y(float y) { _y = y; }

        void z(float z) { _z = z; }
    };
}

#endif // VECTOR3_H

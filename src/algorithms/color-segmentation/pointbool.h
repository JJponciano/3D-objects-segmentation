#include <pcl/point_types.h>

#ifndef POINTBOOL_H
#define POINTBOOL_H

namespace clstr
{
    class PointBool : public pcl::PointXYZRGB
    {
    public:
        PointBool();
        bool getVisited() const{return this->visited;}
        void setVisited(bool visited) {this->visited = visited;}
    private:
        bool visited = false;
    };
}
#endif // POINTBOOL_H

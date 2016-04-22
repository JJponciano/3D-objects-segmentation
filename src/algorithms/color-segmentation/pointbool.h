#include <pcl/point_types.h>
#include <vector>

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
        void addNeighbour(clstr::PointBool* neighbour);
        std::vector<clstr::PointBool*>::iterator getFirstNghbr() { return this->neighbours.begin(); }
        std::vector<clstr::PointBool*>::iterator getEndNghbr() { return this->neighbours.end(); }
    private:
        bool visited = false;
        std::vector<clstr::PointBool*> neighbours;
    };
}
#endif // POINTBOOL_H

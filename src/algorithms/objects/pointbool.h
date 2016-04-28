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

        /**
         * @brief Adds a pointer to a neighbour into the neighbours vector
         * @param neighbour The pointer to the neighbour
         **/
        void addNeighbour(clstr::PointBool* neighbour);

        /**
         * @brief getFirstNghbr returns an iterator to the beggining of the neighbours vector
         * @return an iterator to the beggining of the neighbours vector
         */
        std::vector<clstr::PointBool*>::iterator getFirstNghbr() { return this->neighbours.begin(); }
        std::vector<clstr::PointBool*>::iterator getEndNghbr() { return this->neighbours.end(); }
    private:
        bool visited = false;
        /**
         * @brief neighbours contains the pointers to each neighbours of the point
         */
        std::vector<clstr::PointBool*> neighbours;
    };
}
#endif // POINTBOOL_H

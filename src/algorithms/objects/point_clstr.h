#include <pcl/point_types.h>
#include <vector>

#ifndef POINT_CLSTR_H
#define POINT_CLSTR_H

class bounding_box;

namespace clstr
{
    class point_clstr : public pcl::PointXYZRGB
    {
    public:
        point_clstr();
        point_clstr(float x, float y, float z);
        bool getVisited() const{return this->visited;}
        void setVisited(bool visited) {this->visited = visited;}

        /**
         * @brief Adds a pointer to a neighbour into the neighbours vector
         * @param neighbour The pointer to the neighbour
         **/
        void addNeighbour(clstr::point_clstr* neighbour) { this->neighbours.push_back(neighbour); }

        void makeVertex(){ this->nbTimeVertex++; }
        void removeVertex(){ this->nbTimeVertex--; }

        /**
         * @brief getFirstNghbr returns an iterator to the beggining of the neighbours vector
         * @return an iterator to the beggining of the neighbours vector
         */
        std::vector<clstr::point_clstr*>::iterator getFirstNghbr() { return this->neighbours.begin(); }
        std::vector<clstr::point_clstr*>::iterator getEndNghbr() { return this->neighbours.end(); }
    private:
        bool visited = false;
        /**
         * @brief neighbours contains the pointers to each neighbours of the point
         */
        std::vector<clstr::point_clstr*> neighbours;
        int nbTimeVertex = 0;
    };
}
#endif // point_clstr_H

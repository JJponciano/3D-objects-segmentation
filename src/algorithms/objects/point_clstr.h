#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <list>

#ifndef POINT_CLSTR_H
#define POINT_CLSTR_H

class bounding_box;

namespace clstr
{
    class point_clstr : public pcl::PointXYZRGB
    {
    public:
        /**
         * @brief point_clstr Default constructor
         */
        point_clstr();
        /**
         * @brief point_clstr Constructor that allows to choose the points coordinates and its colour
         * @param x Coordinate X
         * @param y Coordinate Y
         * @param z Coordinate Z
         * @param r Red value
         * @param g Green value
         * @param b Blue value
         */
        point_clstr(float x, float y, float z, float r, float g, float b);

        /**
         * @brief makeVertex Increment the nbTimeVertex variable so the point knows it is the vertex of one box
         */
        void makeVertex(){ this->nbTimeVertex++; }
        /**
         * @brief removeVertex Decrements the nbTimeVertex variable so the point knows it is no longer a vertex of one box
         */
        void removeVertex(){ this->nbTimeVertex--; }
        /**
         * @brief getNbTimeVertex Gets the number of time the point has been made a vertex of a box
         * @return the nbTimeVertex variable
         */
        int getNbTimeVertex(){ return this->nbTimeVertex; }

        /**
         * @brief addNeighbour Adds a reference to a point into the list of this point's neighbours
         * @param point_ptr Reference to the neighbour
         */
        void addNeighbour(clstr::point_clstr* point_ptr) { this->neighbours.push_front(point_ptr); }
        /**
         * @brief getIteratorOnFirstNeighbour Gets an iterator on the first neighbour of this point
         * @return A list iterator on the head of the list
         */
        std::list<clstr::point_clstr*>::iterator getIteratorOnFirstNeighbour() { return this->neighbours.begin(); }
        /**
         * @brief getIteratorOnLastNeighbour gets an iterator on the last neighbour of this point
         * @return A list iterator on the back of the list
         */
        std::list<clstr::point_clstr*>::iterator getIteratorOnLastNeighbour() { return this->neighbours.end(); }

        /**
         * @brief Gets if the point has been visited
         */
        bool getVisited() { return this->visited; }
        /**
         * @brief setVisited Set the visited variable to true or false
         */
        void setVisited(bool visited) { this->visited = visited; }

        /**
         * @brief getAdded Gets if the point has already been added to a cluster
         */
        bool getAdded() { return this->added; }
        /**
         * @brief setAdded Set the added variable to true or false
         */
        void setAdded(bool added) { this->added = added; }

        /**
         * @brief clearNeighbours Remove all the neighbours from this point's neighbours list
         */
        void clearNeighbours() { this->neighbours.clear(); }

    private:
        /**
         * @brief neighbours List containing references to all this point's neighbours
         */
        std::list<clstr::point_clstr*> neighbours;
        /**
         * @brief visited Variable used to know if we already have visited this point, meaning we don't have to check it again and add its neighbours to the cluster again
         */
        bool visited = false;
        /**
         * @brief added Variable used to know if this point has already been added to a cluster
         */
        bool added = false;

        /**
         * @brief nbTimeVertex Variable used to know how many times this point has been made a vertex from a bounding box
         */
        int nbTimeVertex = 0;
    };
}
#endif // point_clstr_H

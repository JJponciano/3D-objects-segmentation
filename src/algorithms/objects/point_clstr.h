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
        point_clstr();
        point_clstr(float x, float y, float z);

        void makeVertex(){ this->nbTimeVertex++; }
        void removeVertex(){ this->nbTimeVertex--; }

        void addNeighbour(clstr::point_clstr* point_ptr) { this->neighbours.push_front(point_ptr); }
        std::list<clstr::point_clstr*>::iterator getIteratorOnFirstNeighbour() { return this->neighbours.begin(); }
        std::list<clstr::point_clstr*>::iterator getIteratorOnLastNeighbour() { return this->neighbours.end(); }

        void setCloud(pcl::PointCloud<clstr::point_clstr>::Ptr cloud) { this->added_in_cloud = cloud; }
        pcl::PointCloud<clstr::point_clstr>::Ptr getCloud() { return this->added_in_cloud; }

        bool getVisited() { return this->visited; }
        void setVisited(bool visited) { this->visited = visited; }

        bool getAdded() { return this->added; }
        void setAdded(bool added) { this->added = added; }

        void clearNeighbours() { this->neighbours.clear(); }

    private:
        std::list<clstr::point_clstr*> neighbours;
        pcl::PointCloud<clstr::point_clstr>::Ptr added_in_cloud = nullptr;
        bool visited = false;
        bool added = false;

        int nbTimeVertex = 0;
    };
}
#endif // point_clstr_H

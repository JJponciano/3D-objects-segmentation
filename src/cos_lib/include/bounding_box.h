#include <vector>
#include "point_clstr.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

namespace cos_lib
{
    class bounding_box
    {
    public:
        /**
         * @brief bounding_box Creates a bounding box according a PCL point cloud
         * @param cloud The cloud used to make the bounding box
         */
        bounding_box(pcl::PointCloud<point_clstr>::Ptr cloud);
        /**
         * @brief bounding_box Creates a bounding box with the points given in parameter
         */
        bounding_box(point_clstr* A,point_clstr* B,point_clstr* C,point_clstr* D,point_clstr* E,point_clstr* F,point_clstr* G,point_clstr* H);
        /**
         * @brief divideBox Divide the bounding box into 8 boxes
         * @return A vector containing a reference to 8 boxes
         */
        std::vector<bounding_box*> divideBox();
        /**
         * @brief addPointIntoBox Used to tell a box if it surrounds a point by adding the points it surronds in it
         * @param point The point you want to add in the box
         */
        void addPointIntoBox(point_clstr* point);
        /**
         * @brief getNbOfPoints Gets how many points are in a box
         * @return returns the number fo points a box surrounds
         */
        int getNbOfPoints() { return this->points.size(); }
        /**
         * @brief getVertices RGets all the box's vertices
         * @return A vectopr containing the 8 vertices of the box
         */
        std::vector<point_clstr*> getVertices();
        /**
         * @brief deleteBox Destroy the box
         */
        void deleteBox();
    private:
        point_clstr* A;
        point_clstr* B;
        point_clstr* C;
        point_clstr* D;
        point_clstr* E;
        point_clstr* F;
        point_clstr* G;
        point_clstr* H;
        /**
         * @brief points Vector containing a reference to the points a box surrounds
         */
        std::vector<point_clstr*> points;
        /**
         * @brief boxes Vector containing all the divided boxes from this box (after using the divideBox function)
         */
        std::vector<bounding_box*> boxes;

    };
}

#endif // BOUNDING_BOX_H

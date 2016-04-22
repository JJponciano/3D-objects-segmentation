#include <map>
#include <vector>
#include <iterator>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/impl/io.hpp>
#include <pointbool.h>
#include "pcloud_io.h"

#ifndef CLUSTERING_H
#define CLUSTERING_H

namespace clstr{

    class clustering
    {
    public:
        clustering();
        //This method returns a vector of the clouds obtained with the colored cloud input as a parameter
        static void getCloudsByColor(pcl::PointCloud<clstr::PointBool>::Ptr base_cloud, double radius, int min_cloud_size = 1000);
        static void convertXYZRGBToBool(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB, pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool);
        static void convertBoolToXYZRGB(pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB);
    private:
        static std::map<uint32_t, pcl::PointCloud<clstr::PointBool>::Ptr> color_map;
        static std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> resulting_clouds;

        //Sort the points of our base cloud into clouds that only contain one color each
        static void sortPointsByColor(pcl::PointCloud<clstr::PointBool>::iterator cloud_iterator);
        static void setNeighbourhood(pcl::PointCloud<clstr::PointBool>::Ptr colored_cloud, double search_radius);
        static void creatingNewCloud(clstr::PointBool* crt_point, pcl::PointCloud<clstr::PointBool>::Ptr);
        static void createTxtFiles(std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clouds);
        /*
        //Arrange the previous clouds into K-d Trees so the points are now sorted by color and coordinates
        static pcl::KdTreeFLANN<clstr::PointBool> getCorrespondingKdTree(pcl::PointCloud<clstr::PointBool>::Ptr colored_cloud);

        /*Using a K-d tree and a radius, we group all the neighbours of a series of points into a new cloud
          so we only have clouds containing same colored and packed points
        static std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> getCloudsFromKDTree(pcl::KdTreeFLANN<clstr::PointBool> kdtree, double search_radius);
        static void addSurroundingPointsToCloud(int PointID, double search_radius, pcl::KdTreeFLANN<clstr::PointBool> kdtree, pcl::PointCloud<clstr::PointBool>::Ptr base_colored_cloud, pcl::PointCloud<clstr::PointBool>::Ptr new_cloud);
        */
    };
}
#endif // CLUSTERING_H

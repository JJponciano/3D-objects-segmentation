#include <map>
#include <vector>
#include <iterator>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/impl/io.hpp>
#include <./objects/pointbool.h>
#include "./io/pcloud_io.h"
#include "./cloud_manip/cloud_manip.h"

#ifndef CLUSTERING_H
#define CLUSTERING_H

namespace clstr{

    class clustering
    {
    public:
        /**
         * @brief Segment a multi-coloured cloud into multiple uni-coloured clouds
         * @param base_cloud The multi-coloured cloud that needs to be segmented
         * @param radius The radius to find each points neighbour (the bigger the radius is the less precise the obtained clouds are)
         * @param min_cloud_size Default 1000 points. Defines how many points the clouds obtained should at least have
         **/
        static std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> getCloudsByColor(pcl::PointCloud<clstr::PointBool>::Ptr base_cloud, double radius, size_t min_cluster_size = 1000);
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudFromVector(std::vector<pcl::PointCloud<clstr::PointBool>::Ptr>);
    private:
        /**
         * @brief color_map <map> that has RGB values as keys and return the corresponding coloured cloud
         */
        static std::map<std::string, pcl::PointCloud<clstr::PointBool>::Ptr> color_map;
        /**
         * @brief resulting_clouds Stores the clouds resulting from the segmentation
         */
        static std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> resulting_clouds;

        /**
         * @brief sortPointsByColor Sort each point into the <color-map> depending on their RGB value
         * @param cloud_iterator An iterator to the point to be sorted
         */
        static void sortPointsByColor(pcl::PointCloud<clstr::PointBool>::iterator cloud_iterator);

        /**
         * @brief setNeighbourhood Gets each point neighbours
         * @param colored_cloud The cloud in which the researches are done
         * @param search_radius The radius by which the neighbours are found around the currently looked at point
         */
        static void setNeighbourhood(pcl::PointCloud<clstr::PointBool>::Ptr colored_cloud, double search_radius);

        /**
         * @brief createNewCluster Despites its name, adds a whole neighbourhood into a new cloud
         * @param crt_point Pointer to the point we want to find the neighbours and add them
         * @param new_cluster Cloud in which the neighbourhood will be stored
         */
        static void createNewCluster(clstr::PointBool* crt_point, pcl::PointCloud<clstr::PointBool>::Ptr new_cluster);

        /**
         * @brief createTxtFiles Create one text file per cloud found
         * @param clouds A vector containing each of the clouds obtained as a result of the segmentation
         */
        static void createTxtFiles(std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clouds);

        static int roundToNearestTenth(int i);
    };
}
#endif // CLUSTERING_H

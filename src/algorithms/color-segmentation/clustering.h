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
        /**
         * @brief Segment a multi-coloured cloud into multiple uni-coloured clouds
         * @param base_cloud The multi-coloured cloud that needs to be segmented
         * @param radius The radius to find each points neighbour (the bigger the radius is the less precise the obtained clouds are)
         * @param min_cloud_size Default 1000 points. Defines how many points the clouds obtained should at least have
         **/
        static void getCloudsByColor(pcl::PointCloud<clstr::PointBool>::Ptr base_cloud, double radius, size_t min_cloud_size = 1000);

        /**
         * @brief convertXYZRGBToBool converts a XYZRGB point cloud into a PointBool one (needed to run the <color-segmentation> algorithm)
         * @param cloud_RGB The cloud that needs to be converted
         * @param cloud_bool The cloud in which we store the result
         */
        static void convertXYZRGBToBool(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB, pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool);

        /**
         * @brief convertBoolToXYZRGB converts a PointBool point cloud into a XYZRGB one
         * @param cloud_bool The cloud that needs to be converted
         * @param cloud_RGB The cloud in which we store the result
         */
        static void convertBoolToXYZRGB(pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB);
    private:
        /**
         * @brief color_map <map> that has RGB values as keys and return the corresponding coloured cloud
         */
        static std::map<uint32_t, pcl::PointCloud<clstr::PointBool>::Ptr> color_map;
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
         * @brief createNewCloud Despites its name, adds a whole neighbourhood into a new cloud
         * @param crt_point Pointer to the point we want to find the neighbours and add them
         * @param new_cloud Cloud in which the neighbourhood will be stored
         */
        static void createNewCloud(clstr::PointBool* crt_point, pcl::PointCloud<clstr::PointBool>::Ptr new_cloud);

        /**
         * @brief createTxtFiles Create one text file per cloud found
         * @param clouds A vector containing each of the clouds obtained as a result of the segmentation
         */
        static void createTxtFiles(std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clouds);
    };
}
#endif // CLUSTERING_H

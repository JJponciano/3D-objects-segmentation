#include <map>
#include <vector>
#include <iterator>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/impl/io.hpp>
#include "point_clstr.h"
#include "../include/cloud_io.h"
#include "cloud_manip.h"
#include "bounding.h"

#ifndef CLUSTERING_H
#define CLUSTERING_H


namespace cos_lib
{
    class clustering
    {
    public:
        /**
         * @brief getClustersFromColouredCloud Retuns a vector containing the coloured clusters found in cloud.
         * @param cloud PCL Cloud with XYZRGB points in which you want to find the clusters
         * @param neighbours_radius Radius of search, used to define at least how near two neighbour points must be from eachother
         * @param isWidop If the cloud to analyse is a Widop cloud
         * @param min_cluster_size Minimum of points a cluster must have
         * @return A vector that contains the clusters found
         */
        static std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getClustersFromColouredCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double neighbours_radius, bool isWidop = true, size_t min_cluster_size = 1000);
    private:
        /**
         * @brief coloured_clouds_map Map that, for a string representing a colour, associate a plane coloured cloud
         */
        static std::map<std::string, pcl::PointCloud<cos_lib::point_clstr>::Ptr> coloured_clouds_map;
        /**
         * @brief neighboursNotYetPushed Vector that remembers which points are yet to be seen during the adding of the neighbours
         */
        static std::vector<cos_lib::point_clstr*> neighboursNotYetPushed;

        /**
         * @brief sortPointsByColor Sorts all the cloud's points into the coloured_cloud_map according to their colour
         * @param base_cloud The cloud we want its points to be sorted
         */
        static void sortPointsByColor(pcl::PointCloud<cos_lib::point_clstr>::Ptr base_cloud);
        /**
         * @brief setNeighbourhoodForPoints With a view to not keep a kd-tree along the whole algorithm, this function tells each points who their neighbours is
         * @param unicoloured_cloud Cloud that is plane coloured
         * @param neighbours_radius Search radius for the neighbours
         */
        static void setNeighbourhoodForPoints(pcl::PointCloud<cos_lib::point_clstr>::Ptr unicoloured_cloud, double neighbours_radius);
        /**
         * @brief addNeighboursIntoCluster Add the current point and its neighbours into the cluster
         * @param point_ptr The current point you want to add
         * @param cluster The cluster you want to push the point in
         * @return A vector that contains the points neighbours that hasn't been seen yet
         */
        static std::vector<cos_lib::point_clstr*> addNeighboursIntoCluster(cos_lib::point_clstr* point_ptr, pcl::PointCloud<cos_lib::point_clstr>::Ptr cluster);
    };
}
#endif // CLUSTERING_H

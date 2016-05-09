#include <map>
#include <vector>
#include <iterator>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/impl/io.hpp>
#include <./objects/point_clstr.h>
#include "./io/pcloud_io.h"
#include "./cloud_manip/cloud_manip.h"
#include <time.h>

#ifndef CLUSTERING_H
#define CLUSTERING_H

namespace clstr{

    class clustering
    {
    public:
        static int getClustersFromColouredCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double neighbours_radius, bool isWidop = true, int min_cluster_size = 1000);
    private:
        static std::map<std::string, pcl::PointCloud<clstr::point_clstr>::Ptr> coloured_clouds_map;
        static std::vector<clstr::point_clstr*> neighboursNotYetPushed;

        static void sortPointsByColor(pcl::PointCloud<clstr::point_clstr>::Ptr base_cloud);
        static void setNeighbourhoodForPoints(pcl::PointCloud<clstr::point_clstr>::Ptr unicoloured_cloud, double neighbours_radius);
        static std::vector<clstr::point_clstr*> addNeighboursIntoCluster(clstr::point_clstr* point_ptr, pcl::PointCloud<clstr::point_clstr>::Ptr cluster);
    };
}
#endif // CLUSTERING_H

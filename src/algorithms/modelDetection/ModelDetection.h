#ifndef LINEFINDING_H
#define LINEFINDING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>

namespace ModelDetection{
     /**
     * @brief coloringProcess Color points given in of the point cloud.
     * @param cloud original point cloud.
     * @param inliers point index to be colored in the point cloud.
     * @param color color in RGB.
     */
    void coloringProcess(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> inliers, std::vector<int> color);


    /**
     * @brief getBestPlan Get the best plan in a cloud
     * @param cloud original point cloud
     * @param distanceThreshold distance threshold to the detection
     * @return list of point index of the best plan
     */
    std::vector<int> getBestPlan(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,double distanceThreshold=0.01);
    /**
     * @brief colorPlans Color all plans in a cloud
     * @param cloud original point cloud
     * @param distanceThreshold distance threshold to the plan detection
     */
    void colorPlans(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,double distanceThreshold=0.01);

    /**
     * @brief getBestLine Get the best line in a cloud
     * @param cloud original point cloud
     * @param distanceThreshold distance threshold to the detection
     * @return list of point index of the best line
     */
    std::vector<int> getBestLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,double distanceThreshold=0.01);
    /**
     * @brief colorPlans Color all lines in a cloud
     * @param cloud original point cloud
     * @param distanceThreshold distance threshold to the plan detection
     */
    void colorLines(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,double distanceThreshold=0.01);

    /**
     * @brief removeSetOfIndices remove points in a cloud
     * @param cloud cloud to remove point
     * @param indices index of point to be removed
     * @return the new cloud without points given.
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeSetOfIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices);

    /**
     * @brief colorRandomizer Get a random color
     * @return random color in RGB
     */
    std::vector<int> colorRandomizer();


   

}

#endif // LINEFINDING_H

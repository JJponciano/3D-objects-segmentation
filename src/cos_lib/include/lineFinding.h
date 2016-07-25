#ifndef LINEFINDING_H
#define LINEFINDING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include "plane.h"
#include "line.h"

namespace cos_lib
{
    /**
     * @brief lineColoring !!WIP!! supposed to color lines but is too slow and was changed to coloring planes, please look at ModelDetection for a definitive version
     * @param cloud IN the initial point cloud with RGB points
     * @return a point cloud with lines colored
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lineColoring(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    /**
     * @brief coloringOneLine Colors a set of points based on their indices (works with planes, too)
     * @param cloud IN the cloud to color the line in
     * @param inliers IN the indices of the points to color
     * @param color IN the ints representing an RGB color
     */
    void coloringOneLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> inliers, std::vector<int> color);

    /**
     * @brief findBestPlane finds the best plane in a cloud with RANSAC
     * @param cloud IN the cloud to find the plane in
     * @param inliers OUT the indices of the inliers of the plane
     * @param coef OUT the coefficients of the plane found
     * @return a Plane Object
     */
    cos_lib::Plane* findBestPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> inliers, Eigen::VectorXf coef);

    /**
     * @brief removeSetOfIndices removes a set of indices from a point cloud
     * @param cloud IN the cloud from which the points will be removed
     * @param indices IN the indices of the points to remove
     * @return the point cloud without the points
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeSetOfIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices);

    /**
     * @brief findOnePlane extract a sub cloud from the cloud
     * @param cloud IN base cloud
     * @param indices IN indices of the points to extract
     * @return a point cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr findOnePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices);

    /**
     * @brief colorEntirePlane colors an entire point cloud
     * @param cloud IN the cloud to color
     * @param color IN the color to use
     */
    void colorEntirePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> color);

    /**
     * @brief colorRandomizer returns a vector of <R,G,B> ints between 0 & 255
     * @return vector<R,G,B>
     */
    std::vector<int> colorRandomizer();

    /**
     * @brief findALineInYDirection finds a line in the Y direction of the point cloud with rRANSAC
     * @param cloud IN the cloud to look for the line in
     * @return the Line object found
     */
    Line* findALineInYDirection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    /**
     * @brief findLinesInYDirection finds multiple lines in Y direction
     * @param cloud IN the base point cloud
     */
    void findLinesInYDirection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    /**
     * @brief findLinesInClusters looks for lines in clusters of points created by the color segmentation part of the project
     * @param clusters IN vector of Point clouds
     * @return returns a point cloud containing the lines found
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr findLinesInClusters(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters);

    /**
     * @brief findLines finds multiple lines in one point cloud
     * @param cloud IN the base cloud
     * @return return a new cloud containing the lines
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr findLines(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    /**
     * @brief findBestLine finds the best line in one point cloud
     * @param cloud IN base cloud
     * @return Line object found
     */
    Line* findBestLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    /**
     * @brief findIntersections !!WIP!! finds intersections between all the lines in a vector
     * @param lines IN the vector countaining the lines to test
     * @return not finished, returns the angles of the intersections for now
     */
    std::vector<float> findIntersections(std::vector<Line*> lines);
}

#endif // LINEFINDING_H

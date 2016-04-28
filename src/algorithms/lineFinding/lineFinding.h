#ifndef LINEFINDING_H
#define LINEFINDING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include "plane.h"
#include "line.h"

namespace lineFinding{
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > lineColoring(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud);

    void coloringOneLine(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> inliers, std::vector<int> color);

    Plane* findBestPlane(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> inliers, Eigen::VectorXf coef);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > removeSetOfIndices(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> indices);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > findOnePlane(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> indices);

    void colorEntirePlane(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> color);

    std::vector<int> colorRandomizer();

    std::vector<float> minMaxCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud);

    float avgDistanceBetweenPoints(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud);

    Line* findALineInYDirection(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud);

    void findLinesInYDirection(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud);
}

#endif // LINEFINDING_H

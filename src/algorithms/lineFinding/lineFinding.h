#ifndef LINEFINDING_H
#define LINEFINDING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>

namespace lineFinding{
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > lineColoring(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud);

    void coloringOneLine(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> inliers, std::vector<int> color);

    std::vector<int> findBestLine(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > removeSetOfIndices(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> indices);

    std::vector<int> colorRandomizer();
}

#endif // LINEFINDING_H

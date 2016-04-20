#ifndef LINEFINDING_H
#define LINEFINDING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/PointIndices.h>
#include <Eigen/StdVector>
#include <boost/shared_ptr.hpp>
#include <pcl/cloud_iterator.h>
#include <iterator>

namespace lineFinding{
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > lineColoring(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud);

    void coloringOneLine(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> inliers, std::vector<int> color);
}

#endif // LINEFINDING_H

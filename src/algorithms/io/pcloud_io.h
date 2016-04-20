#ifndef PCLOUD_IO_H
#define PCLOUD_IO_H

#include <QTextStream>
#include <QString>
#include <QStringList>
#include <QFile>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <boost/lexical_cast.hpp>

#include <iostream>
#include <sstream>
#include <fstream>
#include <ios>


namespace pcloud_io
{
    // output functions
    void cloud_txt(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);

    // input functions
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr load_cloudtxt(std::string path);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr load_cloud(std::string path);
}

#endif // PCLOUD_IO_H


#ifndef CLOUD_IO_H
#define CLOUD_IO_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <ios>
#include <exception>
#include <type_traits>

#include <QTextStream>
#include <QString>
#include <QStringList>
#include <QFile>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <boost/lexical_cast.hpp>

namespace cloud_io
{
    /**
     * @brief import_cloud_txt generates a cloud from a .txt file
     * @param path is a string representing a unique location in the file system
     * @param is_rgb is true if the text file is supposed to generate an rgb cloud as opposed to a greyscale cloud
     * @return a pointer to the cloud that has been read
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr import_cloud_txt(std::string path, bool is_rgb);

    /**
     * @brief import_cloud generates a cloud from a .txt or .pcd file
     * @param path is a string representing a unique location in the file system
     * @param is_rgb is true if the text file is supposed to generate an rgb cloud as opposed to a greyscale cloud
     * @return the a pointer to the cloud that has been read
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr import_cloud(std::string path, bool is_rgb);

    /**
     * @brief export_cloud writes a point cloud to a text file.
     * @param path is a string representing a unique location in the file system
     * @param cloud_ptr is a pointer to the cloud to be exported
     **/ 
    void export_cloud(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);
}

#endif // CLOUD_IO_H


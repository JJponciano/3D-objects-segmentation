/**
  @author Vlad-Adrian Moglan
  @author Kevin Naudin
  @brief contains cloud in and out functions
  */

#ifndef CLOUD_IO_H
#define CLOUD_IO_H

#include "../except/invalid_path.h"
#include "../except/invalid_cloud_pointer.h"

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
     * @throw invalid_path if failed to open file
     * @return a pointer to the cloud that has been read
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr import_cloud_txt(std::string path);

    /**
     * @brief import_cloud generates a cloud from a .txt or .pcd file
     * @param path is a string representing a unique location in the file system
     * @throw invalid_path if failed to open file
     * @return the a pointer to the cloud that has been read
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr import_cloud(std::string path);

    /**
     * @brief export_cloud writes a point cloud to a text file.
     * @param path is a string representing a unique location in the file system
     * @throw invalid_path if failed to open file
     * @throw invalid_cloud_pointer if cloud_ptr is nullptr
     * @param cloud_ptr is a pointer to the cloud to be exported
     **/ 
    void export_cloud(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);
}

#endif // CLOUD_IO_H


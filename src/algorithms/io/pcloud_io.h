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
#include <exception>


namespace pcloud_io
{
    /**
     * @brief import_cloud_txt generates a cloud from a txt file
     * @param path is the access path of the file
     * @param is_rgb is true if the text file is supposed to generate an rgb cloud as opposed to a greyscale cloud
     * @return the generated cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr import_cloud_txt(std::string path, bool is_rgb);

    /**
     * @brief import_cloud generates a cloud from a txt or pcd file
     * @param path is the access path of the file
     * @param is_rgb true if the text file is supposed to generate an rgb cloud as opposed to a greyscale cloud
     * @return the generated cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr import_cloud(std::string path, bool is_rgb);

    /**
     * @brief export_cloud writes the point cloud to a text file.
     * @param path represents the path and name of the file which will be created.
     * @param pc The point cloud which will be saved.
     **/ 
    void export_cloud(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl);
}

#endif // PCLOUD_IO_H


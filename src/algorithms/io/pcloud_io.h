#ifndef PCLOUD_IO_H
#define PCLOUD_IO_H

#include "../objects/greyscale_image.h"
#include "../objects/point_xy_greyscale.h"

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
#include <type_traits>

namespace pcloud_io
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

    /**
     * @brief export_greyscale_vector exports a greyscale vector to a text file
     * @param path is a string representing a unique location in the file system
     * @param greyscale_vector is the point_xy_greyscale array to be exported
     */
    void export_greyscale_vector(std::string path, std::vector<point_xy_greyscale> greyscale_vector);

    /**
     * @brief export_greyscale_image exports a greyscale image to a file in pgm format
     * @param path is a string representing a unique location in the file system
     * @param magic_number defines the format of the file
     * @param gs_img is the greyscale image to be exported
     */
    void export_greyscale_image(std::string path, std::string magic_number, greyscale_image gs_img);
}

#endif // PCLOUD_IO_H


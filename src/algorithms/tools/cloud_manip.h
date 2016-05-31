/**
  @author Vlad-Adrian Moglan
  @author Kevin Naudin
  @brief contains varied point cloud operations
  */

#ifndef CLOUD_MANIP_H
#define CLOUD_MANIP_H

#include "aux.h"
#include "../2d/point_xy_greyscale.h"
#include "../2d/point_xy_mixed.h"
#include "../objects/point_clstr.h"
#include "../except/invalid_cloud_pointer.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/lexical_cast.hpp>

#include <float.h>
#include <stdexcept>

namespace cloud_manip
{
    /** @return all of the x coordinates found in the parameter cloud */
    std::vector<float> cloud_x_coords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

    /** @return all of the y coordinates found in the parameter cloud */
    std::vector<float> cloud_y_coords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

    /** @return all of the z coordinates found in the parameter cloud */
    std::vector<float> cloud_z_coords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

    /**
     * @brief copy_cloud copies a cloud into another cloud
     * @param src is a pointer to the source cloud
     * @param dest is a pointer to the destination cloud
     * @throw std::invalid_argument if src_ptr or dest_ptr are nullptr
     */
    void copy_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr dest_ptr);

    /**
     * @brief scale_cloud rescales a cloud by rescaling the coordinates of all of its points
     * @param cloud_ptr is a pointer to the point cloud to be modified
     * @param x_scale is the float by which the x coordinate will be multiplied
     * @param y_scale is the float by which the y coordinate will be multiplied
     * @param z_scale is the float by which the z coordinate will be multiplied
     * @throw invalid_cloud_pointer if cloud_ptr is equal to nullptr
     * @throw std::invalid_argument if x_scale, y_scale or z_scale are null
     */
    void scale_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, float x_scale, float y_scale,
                     float z_scale);

    /**
     * @brief crop_cloud removes the points of which the coordinates are beyond a certain threshold
     * @param base_cloud_ptr is a pointer to the point cloud to be treated
     * @param x_thresh is the threshold for the x coordinate
     * @param y_thresh is the threshold for the y coordinate
     * @param z_thresh is the threshold for the z coordinate
     * @throw invalid_cloud_pointer if cloud_ptr is equal to nullptr
     * @return a pointer to the cropped cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr crop_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud_ptr,
                           float x_thresh, float y_thresh, float z_thresh);

    /**
     * @brief homogenize_cloud homogenizes the similar colors within a cloud
     * @details if two points have similar but not identical colors they will be attributed the same color
     * @param cloud a pointer to the point cloud to be homogenized
     * @param epsilon defines the 3 dimensions of the cube used to regroup colors
     * @throw std::invalid_argument if cloud_ptr is equal to nullptr or if epsilon is null
     */
    void homogenize_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, short epsilon);

    /**
     * @brief fragment_cloud breaks a cloud down into smaller pieces along the y axis
     * @details only works on clouds using the y axis to represent depth
     * @param cloud_ptr is a pointer to the point cloud to be fragmented
     * @param max_scaled_fragment_depth is the maximum depth of a fragment taking scale into account
     * @throw invalid_cloud_pointer if cloud_ptr is equal to nullptr
     * @throw std::invalid_argument if max_scaled_fragment_depth is negative or 0
     * @return a vector containing former cloud fragments
     */
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fragment_cloud(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, float max_scaled_fragment_depth);

    /**
     * @brief merge_clouds merges cloud fragments into one cloud
     * @param cloud_fragments is an array of cloud fragments
     * @return a pointer the cloud resulted from merging the cloud fragments
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merge_clouds(
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_fragments);

    /** @return the parameter cloud as an RGB cloud **/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_rgb(pcl::PointCloud<pcl::PointXYZ>::Ptr white_cloud);

    /**
     * @brief cloud_to_greyscale turns a 3D rgb point cloud into a 2D grey scale points vector
     * @param cloud_ptr is a pointer the point cloud to be transformed to greyscale
     * @throw std::invalid_argument if cloud_ptr is equal to nullptr
     * @return an array of 2D greyscale points
     */
     std::vector<point_xy_greyscale> cloud_to_2d_greyscale(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

     /**
      * @brief cloud_to_2d_rgb turns a 3D rgb point cloud into a 2D rgb points vector
      * @param cloud_ptr is a pointer to the point cloud to be transformed
      * @throw std::invalid_argument if cloud_ptr is equal to nullptr
      * @return an array of 2D rgb points
      */
     std::vector<point_xy_rgb> cloud_to_2d_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

     /**
      * @brief cloud_to_2d_rgb turns a 3D rgb point cloud into a 2D mixed points vector
      * @details the image keeps each point's x and y coordinates as well as the rgb value
      * @details the z coordinate is turned into a grey scale value
      * @param cloud_ptr is a pointer to the cloud to be turned into a vector
      * @throw std::invalid_argument if cloud_ptr is equal to nullptr
      * @return an array of 2D mixed points
      */
     std::vector<point_xy_mixed> cloud_to_2d_mixed(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

     /**
     * @brief convertBoolToXYZRGB converts a CLSTR point_clstr cloud into a PCL RGB cloud, so its point can be written in a file
     * @param cloud_bool the CLSTR point_clstr from where the data are taken
     * @param cloud_RGB the PCL RGB cloud cloud where the data will be converted
     */
    void convertBoolToXYZRGB(pcl::PointCloud<clstr::point_clstr>::Ptr cloud_bool, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB);

    /**
     * @brief convertBoolToXYZRGB converts a PCL RGB cloud into a CLSTR point_clstr cloud, used for the clustering and bounding algorithm
     * @param cloud_bool the CLSTR point_clstr cloud where the data will be converted
     * @param cloud_RGB the PCL RGB cloud from where the data are taken
     */
    void convertXYZRGBToBool(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB, pcl::PointCloud<clstr::point_clstr>::Ptr cloud_bool);

    void giveRandomColorToCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


}

#endif // CLOUD_MANIP_H


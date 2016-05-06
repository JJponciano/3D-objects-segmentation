#ifndef CLOUD_MANIP_H
#define CLOUD_MANIP_H

#include "../geom_op/geom_op.h"
#include "../objects/point_xy_greyscale.h"
#include "../objects/greyscale_image.h"
#include "../objects/point_clstr.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <float.h>
#include <exception>

namespace cloud_manip
{
    /** @return all of the x coordinates found in the parameter cloud */
    std::vector<float> cloud_x_coords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    /** @return all of the y coordinates found in the parameter cloud */
    std::vector<float> cloud_y_coords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    /** @return all of the z coordinates found in the parameter cloud */
    std::vector<float> cloud_z_coords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    /** @return all the x coordinates in the parameter greyscale vector */
    std::vector<float> greyscale_x_coords(std::vector<point_xy_greyscale> greyscale_vector);

    /** @return all the y coordinates in the parameter greyscale vector */
    std::vector<float> greyscale_y_coords(std::vector<point_xy_greyscale> greyscale_vector);

    /**
     * @brief copy_cloud copies a cloud into another cloud
     * @param src is a pointer to the source cloud
     * @param dest is a pointer to the dest cloud
     */
    void copy_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dest);

    /**
     * @brief scale_cloud rescales a widop cloud by modifying the coordinates of its
     * @param cloud is the pointer to the point cloud to be modified
     * @param x_scale is the float used to scale the x coordinate of the cloud
     * @param y_scale is the float used to scale the y coordinate of the cloud
     * @param z_scale is the float used to scale the z coordinate of the cloud
     * @param precision is the precision for float comparison
     */
    void scale_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     float x_scale,
                     float y_scale,
                     float z_scale,
                     float precision);

    /**
     * @brief fragment_cloud divises a cloud into smaller parts based on the y coordinate of the points
     * @param cloud is the point cloud to be fragmented
     * @param y_scale is the float used to scale the y coordinate of a different type of cloud
     * @param precision is the precision for the float comparison
     * @return a vector containing former cloud fragments
     */
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fragment_cloud(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
            float y_scale,
            float precision);


    /**
     * @brief crop_cloud removes the points of which the coordinates are beyond a certain threshold
     * @param cloud is the point cloud to be treated
     * @param x_thresh is the threshold of the x coordinate
     * @param y_thresh is the threshold of the y coordinate
     * @param z_thresh is the threshold of the z coordinate
     * @param precision is the precision for the float comparison
     * @return a cropped point cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr crop_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           float x_thresh, float y_thresh, float z_thresh, float precision);

    /**
     * @brief merge_clouds merges cloud fragments into a sole cloud
     * @param fragments contains the set of cloud fragments
     * @return the assembled cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merge_clouds(
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fragments);

    /**
     * @brief cloud_to_greyscale turns a 3D colored cloud into a 2D greyscale points vector
     * @param cloud is the point cloud to be transformed to greyscale
     * @param min_z is used for the mapping function
     * @param max_z is used for the mapping function
     * @return an array of 2D greyscale points
     */
     std::vector<point_xy_greyscale> cloud_to_greyscale(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

     /**
      * @brief greyscale_to_image converts a vector of 2D greyscale points to a depth image
      * @param greyscale_vector is the vector containing the 2D greyscale points
      * @param x_epsilon helps delimiting the x cases of the image
      * @return the depth image
      */
     greyscale_image greyscale_to_image(std::vector<point_xy_greyscale> greyscale_vector, float x_epsilon);

     /**
      * @brief cloud_homogenization homogenizes the similar colors within a cloud
      * @details if two points have similar but not identical colors they will be attributed the same color
      * @param cloud the point cloud to be homogenized
      * @param epsilon defines the 3 dimensions of the cube used to regroup colors
      */
     void cloud_homogenization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, short epsilon);

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


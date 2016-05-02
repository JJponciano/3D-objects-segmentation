#ifndef CLOUD_MANIP_H
#define CLOUD_MANIP_H

#include "../geom_op/geom_op.h"

#include "../objects/pointbool.h"
#include "../objects/greyscale_image.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <float.h>
#include <exception>

namespace cloud_manip
{
    /**
     * @brief copy_cloud copies a cloud into another cloud
     * @param src is a pointer to the source cloud
     * @param dest is a pointer to the dest cloud
     */
    void copy_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dest);

    /**
     * @brief scale_cloud rescales a widop cloud by modifying the coordinates of its
     * @param pt_cl is the pointer to the point cloud to be modified
     * @param x_scale is the float used to scale the x coordinate of the cloud
     * @param y_scale is the float used to scale the y coordinate of the cloud
     * @param z_scale is the float used to scale the z coordinate of the cloud
     * @param precision is the precision for float comparison
     */
    void scale_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl,
                     float x_scale,
                     float y_scale,
                     float z_scale,
                     float precision);

    /**
     * @brief fragment_cloud divises a cloud into smaller parts based on the y coordinate of the points
     * @param pt_cl is the point cloud to be fragmented
     * @param y_scale is the float used to scale the y coordinate of a different type of cloud
     * @param precision is the precision for the float comparison
     * @return a vector containing former cloud fragments
     */
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fragment_cloud(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl,
            float y_scale,
            float precision);


    /**
     * @brief crop_cloud removes the points of which the coordinates are beyond a certain threshold
     * @param pt_cl is the point cloud to be treated
     * @param x_thresh is the threshold of the x coordinate
     * @param y_thresh is the threshold of the y coordinate
     * @param z_thresh is the threshold of the z coordinate
     * @param precision is the precision for the float comparison
     * @return a cropped point cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr crop_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl,
                           float x_thresh, float y_thresh, float z_thresh, float precision);

    /**
     * @brief merge_clouds merges cloud fragments into a sole cloud
     * @param fragments contains the set of cloud fragments
     * @return the assembled cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merge_clouds(
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fragments);

    /**
     * @brief color_to_greyscale turns a 3D colored cloud into a 2D greyscale image
     * @param pt_cl is the point cloud to be transformed
     * @param min_z is used for the mapping function
     * @param max_z is used for the mapping function
     * @return the 2D greyscale image
     */
     greyscale_image color_to_greyscale(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl,
            float min_z, float max_z);
     /**W
     * @brief convertBoolToXYZRGB converts a CLSTR PointBool cloud into a PCL RGB cloud, so its point can be written in a file
     * @param cloud_bool the CLSTR PointBool from where the data are taken
     * @param cloud_RGB the PCL RGB cloud cloud where the data will be converted
     */
	void convertBoolToXYZRGB(pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB);

    /**
     * @brief convertBoolToXYZRGB converts a PCL RGB cloud into a CLSTR PointBool cloud, used for the clustering and bounding algorithm
     * @param cloud_bool the CLSTR PointBool cloud where the data will be converted
     * @param cloud_RGB the PCL RGB cloud from where the data are taken
     */
    void convertXYZRGBToBool(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB, pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool);

    /**
     * @brief giveRandomColorToCloud gets a cluster and changes its colour to a random colour
     * @param cloud the cluster you want the colour to change
     */
    void giveRandomColorToCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
}

#endif // CLOUD_MANIP_H


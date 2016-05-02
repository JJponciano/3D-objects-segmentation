#ifndef CLOUD_MANIP_H
#define CLOUD_MANIP_H

#include "../geom_op/geom_op.h"

<<<<<<< HEAD
=======
#include "../objects/pointbool.h"

>>>>>>> 6d71749c856435460ce85af764e76e9e7199a1db
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
<<<<<<< HEAD
=======

	void convertBoolToXYZRGB(pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB);

    void giveRandomColorToCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    void convertXYZRGBToBool(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB, pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool);
>>>>>>> 6d71749c856435460ce85af764e76e9e7199a1db
}

#endif // CLOUD_MANIP_H


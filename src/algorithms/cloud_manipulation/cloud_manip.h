#ifndef CLOUD_MANIP_H
#define CLOUD_MANIP_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cfloat>

#include "../objects/pointbool.h"

#define Y_SCALE 100

namespace cloud_manip
{
    /**
     * @brief widop_to_cloud rescales a widop cloud by modifying the coordinates of its
     * @param pt_cl is the pointer to the point cloud to be modified
     * @param y_scal is the float used to scale the y coordinate of the widop cloud
     */
    void widop_to_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl, float y_scale);

    /**
     * @brief cloud_to_widop rescales a normal cloud to its former widop shape
     * @param pt_cl is the pointer to the point cloud to be modified
     * @param y_scal is the float used to scale the y coordinate of the widop cloud
     */
    void cloud_to_widop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl, float y_scale);

    /**
     * @brief fragment_cloud divises a cloud into smaller parts based on the y coordinate of the points
     * @param pt_cl is the point cloud to be fragmented
     * @param y_scal is the float used to scale the y coordinate of the widop cloud
     * @return a vector of smaller point clouds
     */
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fragment_cloud(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl, float y_scale);

    /**
     * @brief merge_clouds merges cloud fragments into a sole cloud
     * @param fragments contains the set of cloud fragments
     * @return the assembled cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merge_clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fragments);

    /**
     * @brief convertXYZRGBToBool converts a XYZRGB point cloud into a PointBool one (needed to run the <color-segmentation> algorithm)
     * @param cloud_RGB The cloud that needs to be converted
     * @param cloud_bool The cloud in which we store the result
     */
    void convertXYZRGBToBool(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB, pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool);

    /**
     * @brief convertBoolToXYZRGB converts a PointBool point cloud into a XYZRGB one
     * @param cloud_bool The cloud that needs to be converted
     * @param cloud_RGB The cloud in which we store the result
     */
    void convertBoolToXYZRGB(pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB);

    void giveRandomColorToCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
}

#endif // CLOUD_MANIP_H


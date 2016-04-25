#ifndef CLOUD_MANIP_H
#define CLOUD_MANIP_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define Y_SCALE 1000

namespace cloud_manip
{
    /**
     * @brief widop_to_cloud rescales a widop cloud by modifying the coordinates of its
     * @param pt_cl is the pointer to the point cloud to be modified
     */
    void widop_to_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl);

    /**
     * @brief cloud_to_widop rescales a normal cloud to its former widop shape
     * @param pt_cl is the pointer to the point cloud to be modified
     */
    void cloud_to_widop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl);
}

#endif // CLOUD_MANIP_H


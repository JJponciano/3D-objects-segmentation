#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pointbool.h"
#include <pcl/io/pcd_io.h>
#include "clustering.h"
#include <time.h>
#include <iostream>


int main(int argc, char *argv[])
{
    clock_t clk = clock();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZRGB = pcloud_io::load_cloud("/home/hugo/Desktop/widop/txt_tabtest.txt", true);
    pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool (new pcl::PointCloud<clstr::PointBool>);
    clstr::clustering::convertXYZRGBToBool(cloud_XYZRGB, cloud_bool);
    clstr::clustering::getCloudsByColor(cloud_bool, 0.01);
    std::cout << "Program ran in " << (clock()-clk)/CLOCKS_PER_SEC << " seconds." << std::endl;
    return 0;
}

#include <QCoreApplication>
#include <QFile>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pointbool.h"
#include <pcl/io/pcd_io.h>
#include "clustering.h"
#include <time.h>
#include <iostream>
#include <string>
#include "pcloud_io.h"


int main(int argc, char *argv[])
{
    /*pcl::PointCloud<clstr::PointBool>::Ptr base_cloud ( new pcl::PointCloud<clstr::PointBool>);
    base_cloud->width = 1000;
    base_cloud->height = 100;
    base_cloud->points.resize(base_cloud->width*base_cloud->height);

    std::cout << "The cloud contains " << base_cloud->size() << " points." << std::endl;

    for(size_t i=0; i<base_cloud->size(); i++)
    {
        base_cloud->points[i].x= rand() % 50;
        base_cloud->points[i].y= rand() % 50;
        base_cloud->points[i].z= rand() % 50;
        uint8_t r= 255, g= 255, b= rand() % 255;
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        base_cloud->points[i].rgb = rgb;
    }

     clstr::clustering::getCloudsByColor(base_cloud, 1);
     std::cout << "This algorithm ran in " << (float)(clock() - clk)/CLOCKS_PER_SEC << "seconds\n" << std::endl ;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_XYZRGB->width = 2;
    cloud_XYZRGB->height = 1;
    cloud_XYZRGB->resize(cloud_XYZRGB->width * cloud_XYZRGB->height);
    pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool (new pcl::PointCloud<clstr::PointBool>);
    cloud_XYZRGB->points[0].x=0;
    cloud_XYZRGB->points[0].y=0;
    cloud_XYZRGB->points[0].z=0;
    cloud_XYZRGB->points[0].r=0;
    cloud_XYZRGB->points[0].g=0;
    cloud_XYZRGB->points[0].b=0;
    cloud_XYZRGB->points[1].x=1;
    cloud_XYZRGB->points[1].y=1;
    cloud_XYZRGB->points[1].z=1;
    cloud_XYZRGB->points[1].r=1;
    cloud_XYZRGB->points[1].g=1;
    cloud_XYZRGB->points[1].b=1;*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZRGB = pcloud_io::load_cloudtxt("/home/hugo/Desktop/widop/txt_tabtest.txt");
    pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool (new pcl::PointCloud<clstr::PointBool>);
    clstr::clustering::convertXYZRGBToBool(cloud_XYZRGB, cloud_bool);
    clstr::clustering::getCloudsByColor(cloud_bool, 0.1);

    return 0;
}

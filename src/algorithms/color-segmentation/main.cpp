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


int main(int argc, char *argv[])
{
    srand(clock());
    clock_t clk = clock();
    pcl::PointCloud<clstr::PointBool>::Ptr base_cloud ( new pcl::PointCloud<clstr::PointBool>);
    base_cloud->width = 10000000;
    base_cloud->height = 1;
    base_cloud->points.resize(base_cloud->width*base_cloud->height);

    std::cout << "The cloud contains " << base_cloud->size() << " points." << std::endl;

    for(size_t i=0; i<base_cloud->size(); i++)
    {
        base_cloud->points[i].x= rand() % 60 + 1;
        base_cloud->points[i].y= rand() % 60 + 1;
        base_cloud->points[i].z= rand() % 60 + 1;
        uint8_t r=rand() % 255, g=rand() % 255, b=rand() % 255;
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        base_cloud->points[i].rgb = rgb;
    }

     clstr::clustering::getCloudsByColor(base_cloud, 0.1);
     std::cout << "This algorithm ran in " << (float)(clock() - clk)/CLOCKS_PER_SEC << "seconds\n" << std::endl ;
    return 0;
}

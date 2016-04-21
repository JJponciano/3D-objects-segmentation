#define RANGE 0.1

#include <QCoreApplication>

#include "segm.h"
#include "../io/pcloud_io.h"
#include "test_lib.h"

#include <iostream>
#include <time.h>


#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


/* global variables */
// point cloud to be examined
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
std::clock_t t_start;

// pcd file path
std::string pcd_path;

/* function implementation */
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    /*
    // initializing timer
    t_start = -1;

    // initializing cloud
    init_cloud();

    // displaying the menu allowing the user to test functions
    test_menu();*/

    // init_cloud();

    cloud = pcloud_io::load_cloud("/home/vlad-adrian/3D-objects-segmentation/data/table.txt");
    std::cout << cloud->size() << std::endl;
    std::cout << cloud->at(5).rgb << std::endl;

    geom::vectors::estim_normals(cloud, RANGE);

    std::cout << cloud->at(5).rgb << std::endl;

    pcloud_io::cloud_txt("/home/vlad-adrian/Documents/txt_tabtest.txt", cloud);

    std::cout << "done" << std::endl;

    // test_geom::test_crossprod();
    // test_geom::test_translation();

    return a.exec();
}


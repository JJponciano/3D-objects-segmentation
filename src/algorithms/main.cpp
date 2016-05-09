#include <iostream>
#include <vector>
#include "pcl/point_cloud.h"
#include "./objects/point_clstr.h"
#include "./cloud_manip/cloud_manip.h"
#include "./io/pcloud_io.h"
#include "clustering.h"
#include "bounding/bounding.h"

using namespace std;

int test_cloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcloud_io::import_cloud("/home/kevin/Desktop/results_2.txt", true);
    int i = clstr::clustering::getClustersFromColouredCloud(cloud, 0.1, true, 700);
    return i;
}

int roundToNearestTen(int i)
{
    if(i%10<5)i=(i/10)*10;else{i=(i/10)*10+10;}return i;
}

void test_couleur_point()
{
    pcl::PointXYZRGB point;
    point.r = 17;
    point.g = 22;
    point.b = 35;
    int r=(int)point.r,g=(int)point.g,b=(int)point.b;

    r = roundToNearestTen(r);
    g = roundToNearestTen(g);
    b = roundToNearestTen(b);
    std::cout << r << " " << g << " " << b << std::endl;
}

void read_clusters(int j)
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fragments;
    for(int i=1; i<j; i++)
    {
        fragments.push_back(bounding::getCloudBoundings(pcloud_io::import_cloud("/home/kevin/Desktop/build-color-seg-Desktop-Debug/cluster"+std::to_string(i)+".txt", true), 5));
    }
    pcloud_io::export_cloud("final_cloud_bounds.txt" ,cloud_manip::merge_clouds(fragments));
}

void test_bounding()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = pcloud_io::import_cloud("/home/kevin/Desktop/cluster17.txt", true);
    std::cout << "Finished reading cluster"  << std::endl;
    pcloud_io::export_cloud("cluster17_bounds", bounding::getCloudBoundings(cloud_xyzrgb, 6));
}

void assemble_test()
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fragments;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr wall = pcloud_io::import_cloud("/home/kevin/Desktop/boundTest_wall.txt", true);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundings_wall = pcloud_io::import_cloud("/home/kevin/Desktop/bounding_test.txt", true);
    fragments.push_back(wall);
    fragments.push_back(boundings_wall);
    pcloud_io::export_cloud("WALL and BOUNDS.txt" ,cloud_manip::merge_clouds(fragments));
}

int main(int argc, char *argv[])
{
    //test_couleur_point();
    //int i = test_cloud();
    //read_clusters(i);
    //test_bounding();
    assemble_test();
    return 0;
}

#include <iostream>
#include <vector>
#include "pcl/point_cloud.h"
#include "./objects/point_clstr.h"
#include "./cloud_manip/cloud_manip.h"
#include "./io/pcloud_io.h"
#include "clustering.h"

using namespace std;

void test_cloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcloud_io::import_cloud("/home/kevin/Desktop/results_2.txt", true);
    clstr::clustering::getClustersFromColouredCloud(cloud, 0.05, true, 20000);
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

void read_clusters()
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fragments;
    for(int i=1; i<25; i++)
    {
        fragments.push_back(pcloud_io::import_cloud("/home/kevin/Desktop/build-color-seg-Desktop-Debug/cluster"+std::to_string(i)+".txt", true));
    }
    pcloud_io::export_cloud("final_cloud_multicoloured.txt" ,cloud_manip::merge_clouds(fragments));
}

int main(int argc, char *argv[])
{
    //test_couleur_point();
    //test_cloud();
    read_clusters();
}

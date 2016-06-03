#include <iostream>
#include <vector>
#include "pcl/point_cloud.h"
#include "../objects/point_clstr.h"
#include "../tools/cloud_manip.h"
#include "../io/cloud_io.h"
#include "clustering.h"
#include "../bounding/bounding.h"

using namespace std;

void test_cloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_object_segmentation::io::import_cloud("../../results_2.txt");
    clstr::clustering::getClustersFromColouredCloud(cloud, 0.07, true, 700);
}

void assemble_boundings()
{
    int i;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fragments;
    for(i = 1; i < 62; i++)
    {
        fragments.push_back(cloud_object_segmentation::io::import_cloud("home/kevin/Desktop/build-color-seg-Desktop-Debug/bounding"+std::to_string(i)+".txt"));
    }
    //pcloud_io::export_cloud("boundings_assemble.txt", cloud_manip::merge_clouds(fragments));
}

void fonctionJeanJacques()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud = cloud_object_segmentation::io::import_cloud("");
}

int main()
{
    //assemble_boundings();
    test_cloud();
    return 0;
}

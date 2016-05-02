#include <iostream>
#include <vector>
#include "pcl/point_cloud.h"
#include "./objects/pointbool.h"
#include "./cloud_manip/cloud_manip.h"
#include "./io/pcloud_io.h"
#include "clustering.h"

using namespace std;

void test_cloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool (new pcl::PointCloud<clstr::PointBool>);
    std::cout << "Loading Widop file and converting it to cloud format... " << std::flush;
    cloud_xyzrgb = pcloud_io::import_cloud("/home/kevin/Desktop/good_result.txt", true);
    cloud_manip::scale_cloud(cloud_xyzrgb, 1, 100000, 1, 0.0000005);
    std::cout << "Initializing algorithm." << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud = clstr::clustering::getCloudFromVector(clstr::clustering::getCloudsByColor(cloud_bool, 0.05, 200));
    std::cout << "Reconverting to Widop size... " << std::flush;
    cloud_manip::scale_cloud(final_cloud, 1, ((float)1/(float)100000), 1, 0.0000005);
    std::cout << "Writing results into txt file... " << std::flush;
    pcloud_io::export_cloud("test.txt", final_cloud);
    std::cout << "Done." << std::endl;
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

int main(int argc, char *argv[])
{
    //test_couleur_point();
    test_cloud();

    return 0;
}

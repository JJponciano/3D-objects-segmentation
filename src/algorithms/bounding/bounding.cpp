#include "bounding.h"

bounding::bounding()
{

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr bounding::getCloudBoundings(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int iteration)
{
    pcl::PointCloud<clstr::point_clstr>::Ptr mother_cloud (new pcl::PointCloud<clstr::point_clstr>);
    cloud_manip::convertXYZRGBToClstr(cloud, mother_cloud);

    std::vector<bounding_box*> old_boxes;
    std::vector<bounding_box*> new_boxes;
    std::vector<bounding_box*>::iterator boxes_iterator;
    int counter_iter = 0;
    pcl::PointCloud<clstr::point_clstr>::Ptr bounding_cloud (new pcl::PointCloud<clstr::point_clstr>);

    bounding_box* mother_box = new bounding_box(mother_cloud);

    old_boxes.push_back(mother_box);
    while(counter_iter!=iteration && old_boxes.size()<200000)
    {
        for(boxes_iterator=old_boxes.begin(); boxes_iterator!=old_boxes.end(); boxes_iterator++)
        {
            for(bounding_box* box : (*boxes_iterator)->divideBox())
            {
                new_boxes.push_back(box);
            }
        }
        old_boxes.clear();
        old_boxes.shrink_to_fit();
        old_boxes = new_boxes;
        new_boxes.clear();
        new_boxes.shrink_to_fit();
        counter_iter++;
    }
    for(boxes_iterator=old_boxes.begin(); boxes_iterator!=old_boxes.end(); boxes_iterator++)
    {
        for(clstr::point_clstr* point : (*boxes_iterator)->getVertices())
        {
            if(point->getNbTimeVertex() < 2)
            {
                bounding_cloud->push_back(*point);
            }
        }
    }
    old_boxes.clear();
    old_boxes.shrink_to_fit();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_manip::convertClstrToXYZRGB(bounding_cloud, cloud_xyzrgb);
    return cloud_xyzrgb;
}

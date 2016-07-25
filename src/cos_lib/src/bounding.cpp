#include "../include/bounding.h"

cos_lib::bounding::bounding()
{

}

void cos_lib::bounding::getCloudBoundings(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int cluster_number, int iteration)
{
    pcl::PointCloud<point_clstr>::Ptr mother_cloud (new pcl::PointCloud<point_clstr>);
    cos_lib::cloud_manip::convertXYZRGBToClstr(cloud, mother_cloud);

    std::vector<bounding_box*> old_boxes;
    std::vector<bounding_box*> new_boxes;
    std::vector<bounding_box*>::iterator boxes_iterator;
    int counter_iter = 0;
    pcl::PointCloud<point_clstr>::Ptr bounding_cloud (new pcl::PointCloud<point_clstr>);

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

    std::string _filename = "bounding"+std::to_string(cluster_number)+".txt";
    std::remove(_filename.c_str());
    std::ofstream file(_filename, std::ios::app);

    for(boxes_iterator=old_boxes.begin(); boxes_iterator!=old_boxes.end(); boxes_iterator++)
    {
        for(point_clstr* point : (*boxes_iterator)->getVertices())
        {
            file << point->x << " " << point->y << " " << point->z << std::endl;
        }
    }

    file.close();

    old_boxes.clear();
    old_boxes.shrink_to_fit();
}

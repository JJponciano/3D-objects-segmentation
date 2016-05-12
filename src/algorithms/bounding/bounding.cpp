#include "bounding.h"

bounding::bounding()
{

}

pcl::PointCloud<clstr::point_clstr>::Ptr bounding::getCloudBoundings(pcl::PointCloud<clstr::point_clstr>::Ptr cloud, int iteration)
{
    std::vector<bounding_box*> old_boxes;
    std::vector<bounding_box*> new_boxes;

    bounding_box* mother_box = new bounding_box(cloud);
    old_boxes.push_back(mother_box);

    for(int i = 1; i <= iteration; i++)
    {
        for(bounding_box* primary_box : old_boxes)
        {
            for(bounding_box* secondary_box : primary_box->divideBox())
            {
                new_boxes.push_back(secondary_box);
            }
        }
        old_boxes = new_boxes;
        new_boxes.clear();
        new_boxes.shrink_to_fit();
    }

    pcl::PointCloud<clstr::point_clstr>::Ptr bounding_cloud (new pcl::PointCloud<clstr::point_clstr>);

    for(bounding_box* box : old_boxes)
    {
        // If a vertex is the vertex of less than 4 cubes we add it to the cloud
    }

    return bounding_cloud;
}

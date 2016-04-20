/* Author : Kévin Naudin
 * Version : 1.0
 * Made for the I3 Mainz laboratory under GPL license
 */

#include "clustering.h"

//-----------------------------INITIALIZING STATIC ATTRIBUTES----------------------------------------------
std::map<uint32_t, pcl::PointCloud<clstr::PointBool>::Ptr> clstr::clustering::color_map;
std::vector<std::vector<pcl::PointCloud<clstr::PointBool>::Ptr>> clstr::clustering::resulting_clouds;

//------------------------------------PUBLIC METHODS-------------------------------------------------------
clstr::clustering::clustering()
{
    // Do nothing
}

std::vector<std::vector<pcl::PointCloud<clstr::PointBool>::Ptr>> clstr::clustering::getCloudsByColor(pcl::PointCloud<clstr::PointBool>::Ptr base_cloud, double radius)
{
    // We first sort all the points by colour
    std::cout << "Sorting points by their colours" << std::endl;
    pcl::PointCloud<clstr::PointBool>::iterator cloud_iterator;
    for(cloud_iterator = base_cloud->begin(); cloud_iterator!=base_cloud->end(); cloud_iterator++)
    {
        sortPointsByColor(cloud_iterator);
    }
    std::cout << color_map.size() << " different colours have been found" << std::endl;

    //Iterator to go through the entire map
    std::map<uint32_t, pcl::PointCloud<clstr::PointBool>::Ptr>::iterator map_it;
    /* We then create a 3-d Tree from the cloud we built earlier
     * We also set a radius so we can easily find the neighbours of each points
     * And finally we add the neighbourhood in their own cloud
     */
    for(map_it = color_map.begin(); map_it != color_map.end(); map_it++)
    {
        pcl::KdTreeFLANN<clstr::PointBool> kdtree = getCorrespondingKdTree(map_it->second);
        resulting_clouds.push_back(getCloudsFromKDTree(kdtree, radius));
    }
    std::cout << "Finished finding clouds" << std::endl;
    int total_clouds = 0;
    for(std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> vct : resulting_clouds)
    {
        total_clouds += vct.size();
    }
    std::cout << "A total of "<<total_clouds<<" clouds have been found." << std::endl;
    return resulting_clouds;
}

//------------------------------------PRIVATE METHODS-------------------------------------------------------
void clstr::clustering::sortPointsByColor(pcl::PointCloud<clstr::PointBool>::iterator cloud_iterator)
{
    // Get the point color
    uint32_t point_color = (uint32_t)(*cloud_iterator).rgb;
    // Iterator gets to the position of the key if it exists
    std::map<uint32_t, pcl::PointCloud<clstr::PointBool>::Ptr>::iterator map_it = color_map.find(point_color);
    if(map_it != color_map.end())
    {
        // Key already exists so we push our point into the pre-existing cloud
        map_it->second->push_back(*cloud_iterator);
    }
    else
    {
        // Key doesn't exist yet so we create a new cloud for this color and push the point in
        pcl::PointCloud<clstr::PointBool>::Ptr new_colored_cloud (new pcl::PointCloud<clstr::PointBool>);
        color_map[point_color] = new_colored_cloud;
        new_colored_cloud->push_back(*cloud_iterator);
    }
}

pcl::KdTreeFLANN<clstr::PointBool> clstr::clustering::getCorrespondingKdTree(pcl::PointCloud<clstr::PointBool>::Ptr colored_cloud)
{
    // We create a kdtree from our colored_cloud
    pcl::KdTreeFLANN<clstr::PointBool> kdtree;
    kdtree.setInputCloud(colored_cloud);
    return kdtree;
}

//Gets a random point and calls another function to get its neighbourhood into a new cloud
std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clstr::clustering::getCloudsFromKDTree(pcl::KdTreeFLANN<clstr::PointBool> kdtree, double search_radius)
{
    std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clouds_found;
    pcl::PointCloud<clstr::PointBool>::const_iterator cloud_iterator;
    std::vector<int> crtPointID;
    std::vector<float> not_useful; //This attribute is only used because Kdtree needs it to run the radius search algorithm

    for(cloud_iterator=kdtree.getInputCloud()->begin(); cloud_iterator!=kdtree.getInputCloud()->end(); cloud_iterator++)
    {
        crtPointID.clear();
        kdtree.radiusSearch(*cloud_iterator,1,crtPointID,not_useful,1);
        if(!(color_map[kdtree.getInputCloud()->points[crtPointID[0]].rgb])->points[crtPointID[0]].getVisited())
        {
            pcl::PointCloud<clstr::PointBool>::Ptr new_cloud (new pcl::PointCloud<clstr::PointBool>);
            addSurroundingPointsToCloud(crtPointID[0], search_radius, kdtree, color_map[kdtree.getInputCloud()->points[crtPointID[0]].rgb], new_cloud);
            clouds_found.push_back(new_cloud);
        }
    }
    return clouds_found;
}

void clstr::clustering::addSurroundingPointsToCloud(int PointID, double search_radius, pcl::KdTreeFLANN<clstr::PointBool> kdtree, pcl::PointCloud<clstr::PointBool>::Ptr base_colored_cloud, pcl::PointCloud<clstr::PointBool>::Ptr new_cloud)
{
    new_cloud->push_back(base_colored_cloud->points[PointID]);
    base_colored_cloud->points[PointID].setVisited(true);

    std::vector<int> PointsID;
    std::vector<float> not_useful;

    if(kdtree.radiusSearch(base_colored_cloud->points[PointID], search_radius, PointsID, not_useful) > 1)
    {
        for(size_t i=1; i<PointsID.size(); i++)
        {
            if(!kdtree.getInputCloud()->points[PointsID[i]].getVisited())
            {
                addSurroundingPointsToCloud(PointsID[i], search_radius, kdtree, base_colored_cloud, new_cloud);
            }
        }
    }
}

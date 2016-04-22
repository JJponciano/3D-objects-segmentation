/* Author : Kévin Naudin
 * Version : 1.0
 * Made for the I3 Mainz laboratory under GPL license
 */

#include "clustering.h"

//-----------------------------INITIALIZING STATIC ATTRIBUTES----------------------------------------------
std::map<uint32_t, pcl::PointCloud<clstr::PointBool>::Ptr> clstr::clustering::color_map;
std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clstr::clustering::resulting_clouds;

//------------------------------------PUBLIC METHODS-------------------------------------------------------
clstr::clustering::clustering()
{
    // Do nothing
}

void clstr::clustering::getCloudsByColor(pcl::PointCloud<clstr::PointBool>::Ptr base_cloud, double radius, int min_cloud_size)
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
    int counter = 1;
    for(map_it = color_map.begin(); map_it!=color_map.end(); map_it++)
    {
        setNeighbourhood(map_it->second, radius);
        std::cout << "Finished setting neighbourhoods for cloud number " << counter << std::endl;
        pcl::PointCloud<clstr::PointBool>::iterator cloud_it = (map_it->second)->begin();
        for(cloud_it; cloud_it!=(map_it->second)->end(); cloud_it++)
        {
            if(!(*cloud_it).getVisited())
            {
                pcl::PointCloud<clstr::PointBool>::Ptr new_cloud (new pcl::PointCloud<clstr::PointBool>);
                (*cloud_it).setVisited(true);
                new_cloud->push_back(*cloud_it);
                clstr::PointBool* pt_ptr = &(*cloud_it);
                creatingNewCloud(pt_ptr, new_cloud);
                if(new_cloud->size() >= min_cloud_size) resulting_clouds.push_back(new_cloud);
            }
        }
        counter++;
    }
    std::cout << "Found " << resulting_clouds.size() << " clouds" << std::endl;
    createTxtFiles(resulting_clouds);
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

void clstr::clustering::setNeighbourhood(pcl::PointCloud<clstr::PointBool>::Ptr colored_cloud, double search_radius)
{
    pcl::KdTreeFLANN<clstr::PointBool> kdtree;
    kdtree.setInputCloud(colored_cloud);
    pcl::PointCloud<clstr::PointBool>::iterator cloud_it = colored_cloud->begin();
    for(cloud_it; cloud_it!=colored_cloud->end(); cloud_it++)
    {
        std::vector<int> PointsID;
        std::vector<float> distances;
        if(kdtree.radiusSearch(*cloud_it, search_radius, PointsID, distances) > 1)
        {
            for(size_t i = 1; i<PointsID.size(); i++)
            {
                clstr::PointBool* nghbr_ptr = nullptr;
                nghbr_ptr = &(colored_cloud->points[PointsID[i]]);
                (*cloud_it).addNeighbour(nghbr_ptr);
            }
        }
        PointsID.clear();
        PointsID.shrink_to_fit();
        distances.clear();
        distances.shrink_to_fit();
    }
}

void clstr::clustering::creatingNewCloud(clstr::PointBool* crt_point, pcl::PointCloud<clstr::PointBool>::Ptr new_cloud)
{
    std::vector<clstr::PointBool*>::iterator nghbr_it;
    for(nghbr_it = crt_point->getFirstNghbr(); nghbr_it!=crt_point->getEndNghbr(); nghbr_it++)
    {
        if(!(**nghbr_it).getVisited())
        {
            (**nghbr_it).setVisited(true);
            new_cloud->push_back(**nghbr_it);
            creatingNewCloud(*nghbr_it, new_cloud);
        }
    }
}

/*pcl::KdTreeFLANN<clstr::PointBool> clstr::clustering::getCorrespondingKdTree(pcl::PointCloud<clstr::PointBool>::Ptr colored_cloud)
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
        not_useful.clear();
        kdtree.radiusSearch(*cloud_iterator,1,crtPointID,not_useful,1);
        if(!(color_map[kdtree.getInputCloud()->points[crtPointID[0]].rgb])->points[crtPointID[0]].getVisited())
        {
            std::cout << "Found a new cloud." << std::endl;
            pcl::PointCloud<clstr::PointBool>::Ptr new_cloud (new pcl::PointCloud<clstr::PointBool>);
            addSurroundingPointsToCloud(crtPointID[0], search_radius, kdtree, color_map[kdtree.getInputCloud()->points[crtPointID[0]].rgb], new_cloud);
            if(new_cloud->size()>=100) clouds_found.push_back(new_cloud);
            else { std::cout << "cloud was not pushed" << std::endl; }
        }
    }
    crtPointID.clear();
    crtPointID.shrink_to_fit();
    not_useful.clear();
    not_useful.shrink_to_fit();
    //kdtree.~KdTreeFLANN();

    return clouds_found;
}

void clstr::clustering::addSurroundingPointsToCloud(int PointID, double search_radius, pcl::KdTreeFLANN<clstr::PointBool> kdtree, pcl::PointCloud<clstr::PointBool>::Ptr base_colored_cloud, pcl::PointCloud<clstr::PointBool>::Ptr new_cloud)
{
    /*new_cloud->push_back(base_colored_cloud->points[PointID]);
    base_colored_cloud->points[PointID].setVisited(true);

    std::vector<int> *PointsID = new std::vector<int>();
    std::vector<float> *not_useful = new std::vector<float>();

    if(kdtree.radiusSearch(base_colored_cloud->points[PointID], search_radius, *PointsID, *not_useful) > 1)
    {
        for(std::vector<int>::iterator it = PointsID->begin(); it != PointsID->end(); it++)
        {
            if(!kdtree.getInputCloud()->points[*it].getVisited())
            {
                addSurroundingPointsToCloud(*it, search_radius, kdtree, base_colored_cloud, new_cloud);
            }
        }
    }
    PointsID->clear();
    PointsID->shrink_to_fit();
    delete(PointsID);
    not_useful->clear();
    not_useful->shrink_to_fit();
    delete(not_useful);
}*/

void clstr::clustering::convertXYZRGBToBool(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB, pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool)
{
    cloud_bool->width = cloud_RGB->width;
    cloud_bool->height = cloud_RGB->height;
    cloud_bool->resize(cloud_bool->width * cloud_bool->height);
    for(size_t i=0; i<cloud_RGB->points.size(); i++)
    {
        cloud_bool->points[i].x = cloud_RGB->points[i].x;
        cloud_bool->points[i].y = cloud_RGB->points[i].y;
        cloud_bool->points[i].z = cloud_RGB->points[i].z;
        cloud_bool->points[i].rgb = cloud_RGB->points[i].rgb;
        cloud_bool->points[i].setVisited(false);
    }
}

void clstr::clustering::convertBoolToXYZRGB(pcl::PointCloud<clstr::PointBool>::Ptr cloud_bool, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB)
{
    cloud_RGB->width = cloud_bool->width;
    cloud_RGB->height = cloud_bool->height;
    cloud_RGB->resize(cloud_RGB->width * cloud_RGB->height);
    for(size_t i=0; i<cloud_bool->points.size(); i++)
    {
        cloud_RGB->points[i].x = cloud_bool->points[i].x;
        cloud_RGB->points[i].y = cloud_bool->points[i].y;
        cloud_RGB->points[i].z = cloud_bool->points[i].z;
        cloud_RGB->points[i].rgb = cloud_bool->points[i].rgb;
    }
}

void clstr::clustering::createTxtFiles(std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clouds)
{
    std::vector<pcl::PointCloud<clstr::PointBool>::Ptr>::iterator vct_it;
    int counter = 1;
    std::string fileName;
    for(vct_it = clouds.begin(); vct_it!=clouds.end(); vct_it++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB (new pcl::PointCloud<pcl::PointXYZRGB>);
        convertBoolToXYZRGB(*vct_it, cloud_RGB);
        fileName = "cloud";
        fileName += counter;
        fileName += ".txt";
        pcloud_io::cloud_to_txt(fileName, cloud_RGB);
        counter++;
    }
}

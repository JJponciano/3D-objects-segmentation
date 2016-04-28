/* Author : Kévin Naudin
 * Version : 1.0
 * Made for the I3 Mainz laboratory under GPL license
 */

#include "clustering.h"

//-----------------------------INITIALIZING STATIC ATTRIBUTES----------------------------------------------
std::map<uint32_t, pcl::PointCloud<clstr::PointBool>::Ptr> clstr::clustering::color_map;
std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clstr::clustering::resulting_clouds;

//------------------------------------PUBLIC METHODS-------------------------------------------------------

std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clstr::clustering::getCloudsByColor(pcl::PointCloud<clstr::PointBool>::Ptr base_cloud, double radius, size_t min_cloud_size)
{
    // Sorting the points by their RGB value
    std::cout << "Sorting points by their colours" << std::endl;
    pcl::PointCloud<clstr::PointBool>::iterator cloud_iterator;
    for(cloud_iterator = base_cloud->begin(); cloud_iterator!=base_cloud->end(); cloud_iterator++)
    {
        sortPointsByColor(cloud_iterator);
    }
    std::cout << color_map.size() << " different colours have been found" << std::endl;

    // By iterating through the entire <map> we set a neighbourhood for each point of the currently looked at coloured cloud
    // By grouping every neighbourhood we will be able to segment the colour cloud into multiple segmented clouds of the same colour
    // We then check if the point has already been visited. If it's not the case, that means it must belong to another segmented cloud
    std::map<uint32_t, pcl::PointCloud<clstr::PointBool>::Ptr>::iterator map_it;
    //These ints are only used for the informational text written on the console
    int count_map = 1;
    int count_used_map = 1;
    int non_used_clouds = 0;
    int total_size_kept = 0;
    for(map_it = color_map.begin(); map_it!=color_map.end(); map_it++)
    {
        if((map_it->second)->size() >= min_cloud_size)
        {
            setNeighbourhood(map_it->second, radius);
            pcl::PointCloud<clstr::PointBool>::iterator cloud_it;
            for(cloud_it = (map_it->second)->begin(); cloud_it!=(map_it->second)->end(); cloud_it++)
            {
                if(!(*cloud_it).getVisited()) // If the point hasn't been visited yet
                {
                    // We create a new cloud, set the point to visited then add him and its neighbours into the newly created cloud
                    pcl::PointCloud<clstr::PointBool>::Ptr new_cloud (new pcl::PointCloud<clstr::PointBool>);
                    (*cloud_it).setVisited(true);
                    new_cloud->push_back(*cloud_it);
                    clstr::PointBool* pt_ptr = &(*cloud_it);
                    createNewCloud(pt_ptr, new_cloud);
                    if(new_cloud->size()>=min_cloud_size)
                    {
                        resulting_clouds.push_back(new_cloud);
                        total_size_kept += new_cloud->size();
                    }
                    else { non_used_clouds++; }
                }
            }
            std::cout << "New cloud(s) found for color " << count_map << std::endl;
            count_used_map++;
        }
        count_map++;
    }
    std::cout << "Found " << resulting_clouds.size() << " clouds" << std::endl;
    std::cout << "Ignored " << color_map.size()-count_used_map << " colors and " << non_used_clouds << " clouds" << std::endl;
    std::cout << "On a base of " << base_cloud->size() << " points, only " << total_size_kept << " were kept" << std::endl;

    return resulting_clouds;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr clstr::clustering::getCloudFromVector(std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clouds)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointCloud<clstr::PointBool>::Ptr>::iterator vector_it;
    pcl::PointCloud<clstr::PointBool>::iterator clouds_it;
    for(vector_it=clouds.begin(); vector_it!=clouds.end(); vector_it++)
    {
        for(clouds_it=(*vector_it)->begin(); clouds_it!=(*vector_it)->end(); clouds_it++)
        {
            final_cloud->push_back(*clouds_it);
        }
    }

    return final_cloud;

}

//------------------------------------PRIVATE METHODS-------------------------------------------------------
void clstr::clustering::sortPointsByColor(pcl::PointCloud<clstr::PointBool>::iterator cloud_iterator)
{
    // Gets the point color and check if it already exists as a key in our <map>
    // If the key value has been found, we push the point into the cloud that corresponds to the key
    // Else we create a new cloud for a new key value and push the point in this new cloud instead
    uint32_t point_color = (uint32_t)(*cloud_iterator).rgb;
    std::map<uint32_t, pcl::PointCloud<clstr::PointBool>::Ptr>::iterator map_it = color_map.find(point_color);
    if(map_it != color_map.end()) // Means the key already exists
    {
        map_it->second->push_back(*cloud_iterator);
    }
    else
    {
        pcl::PointCloud<clstr::PointBool>::Ptr new_colored_cloud (new pcl::PointCloud<clstr::PointBool>);
        color_map[point_color] = new_colored_cloud;
        new_colored_cloud->push_back(*cloud_iterator);
    }
}

void clstr::clustering::setNeighbourhood(pcl::PointCloud<clstr::PointBool>::Ptr colored_cloud, double search_radius)
{
    // Using a kdtree we are able to sort the points by coordinates
    // Since the clouds used are already sorted by colour, we are sure that each point find in a search radius is a true neighbour to another
    pcl::KdTreeFLANN<clstr::PointBool> kdtree;
    kdtree.setInputCloud(colored_cloud);
    pcl::PointCloud<clstr::PointBool>::iterator cloud_it;
    for(cloud_it = colored_cloud->begin(); cloud_it!=colored_cloud->end(); cloud_it++)
    {
        std::vector<int> PointsID; // Contains the ID of the neighbours
        std::vector<float> distances; // Contains the squared distances of the nieghbours (useless but mandatory)
        if(kdtree.radiusSearch(*cloud_it, search_radius, PointsID, distances) > 1) // Note that 0 refers to the point itself
        {
            for(size_t i = 1; i<PointsID.size(); i++)
            {
                clstr::PointBool* nghbr_ptr = nullptr;
                nghbr_ptr = &(colored_cloud->points[PointsID[i]]);
                (*cloud_it).addNeighbour(nghbr_ptr); // We add a pointer of the neighbour because storing pointer costs way less rapid access memory than a copy of the neighbour itself
            }
        }
        // Deleting a vecotr does not free memory so we clear it and shrink its size to what space it really needs. In this case, the space is of 0.
        PointsID.clear();
        PointsID.shrink_to_fit();
        distances.clear();
        distances.shrink_to_fit();
    }
}

void clstr::clustering::createNewCloud(clstr::PointBool* crt_point, pcl::PointCloud<clstr::PointBool>::Ptr new_cloud)
{
    std::vector<clstr::PointBool*>::iterator nghbr_it;
    // We iterate through the neighbourhood of the point
    for(nghbr_it = crt_point->getFirstNghbr(); nghbr_it!=crt_point->getEndNghbr(); nghbr_it++)
    {
        // If the neighbours hasn't been visited yet
        if(!(**nghbr_it).getVisited())
        {
            // We set it as visited, push it in the cloud and then iterate through its neighbours
            (**nghbr_it).setVisited(true);
            new_cloud->push_back(**nghbr_it);
            createNewCloud(*nghbr_it, new_cloud);
        }
    }
}

void clstr::clustering::createTxtFiles(std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clouds)
{
    // As our IO function does not recognize the point type we created for the <color-segmentation> algorithm, we re-convert our clouds into XYZRGB ones
    std::vector<pcl::PointCloud<clstr::PointBool>::Ptr>::iterator vct_it;
    int counter = 1;
    std::string fileName;
    for(vct_it = clouds.begin(); vct_it!=clouds.end(); vct_it++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB (new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_manip::convertBoolToXYZRGB(*vct_it, cloud_RGB);
        fileName = "cloud";
        fileName += std::to_string(counter);
        fileName += ".txt";
        pcloud_io::cloud_to_txt(fileName, cloud_RGB);
        counter++;
    }
}

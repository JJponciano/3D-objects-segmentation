/* Author : Kévin Naudin
 * Version : 1.0
 * Made for the I3 Mainz laboratory under GPL license
 */

#include "clustering.h"

//-----------------------------INITIALIZING STATIC ATTRIBUTES----------------------------------------------
std::map<std::string, pcl::PointCloud<clstr::PointBool>::Ptr> clstr::clustering::color_map;
std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clstr::clustering::resulting_clouds;

//------------------------------------PUBLIC METHODS-------------------------------------------------------

std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clstr::clustering::getCloudsByColor(pcl::PointCloud<clstr::PointBool>::Ptr base_cloud, double radius, size_t min_cluster_size)
{
    // Sorting the points by their RGB value
    pcl::PointCloud<clstr::PointBool>::iterator cloud_iterator;
    std::cout << "Please wait while the algorithm sorts the cloud colours. " << std::flush;
    for(cloud_iterator = base_cloud->begin(); cloud_iterator!=base_cloud->end(); cloud_iterator++)
    {
        sortPointsByColor(cloud_iterator);
    }
    std::cout << "Approximated values and found " << color_map.size() << " different colours." << std::endl;

    // By iterating through the entire <map> we set a neighbourhood for each point of the currently looked at coloured cloud
    // By grouping every neighbourhood we will be able to segment the colour cloud into multiple segmented clouds of the same colour
    // We then check if the point has already been visited. If it's not the case, that means it must belong to another segmented cloud
    std::map<std::string, pcl::PointCloud<clstr::PointBool>::Ptr>::iterator map_it;
    //These ints are only used for the informational text written on the console
    int non_used_clouds = 0;
    int total_point_size = 0;
    int total_size_kept = 0;
    int colour_counter=0;

    std::cout << "The algorithm will now proceed to find the different clusters. This operation WILL take a while... " << std::flush;
    for(map_it = color_map.begin(); map_it!=color_map.end(); map_it++)
    {
        colour_counter++;
        if((map_it->second)->size() >= min_cluster_size)
        {
            std::cout << "Analyzing colour n°" << colour_counter << std::endl;
            setNeighbourhood(map_it->second, radius);
            pcl::PointCloud<clstr::PointBool>::iterator cloud_it;
            for(cloud_it = (map_it->second)->begin(); cloud_it < (map_it->second)->end(); cloud_it++)
            {
                if(!(*cloud_it).getVisited()) // If the point hasn't been visited yet
                {
                    // We create a new cloud, set the point to visited then add him and its neighbours into the newly created cloud
                    pcl::PointCloud<clstr::PointBool>::Ptr new_cluster (new pcl::PointCloud<clstr::PointBool>);
                    (*cloud_it).setVisited(true);
                    new_cluster->push_back(*cloud_it);
                    clstr::PointBool* pt_ptr = &(*cloud_it);
                    createNewCluster(pt_ptr, new_cluster);
                    if(new_cluster->size()>=min_cluster_size)
                    {
                        resulting_clouds.push_back(new_cluster);
                        total_size_kept += new_cluster->size();
                    }
                    else { non_used_clouds++; new_cluster->clear(); new_cluster->resize(0);}
                    total_point_size += new_cluster->size();
                }
            }
        }
        // Frees memory by erasing the entry
        color_map.erase(map_it);
    }
    std::cout << " Finished finding clusters." << std::endl;
    std::cout << "Found " << resulting_clouds.size() << " clusters." << std::endl;
    std::cout << "The clusters have an average of " << (double)(total_point_size/resulting_clouds.size()) << " points." << std::endl;
    std::cout << "A total of " << non_used_clouds << " clusters were beneath the minimum size required. (" << min_cluster_size << ")" << std::endl;
    std::cout << "Starting with " << base_cloud->size() << " points, only " << total_size_kept << " were kept" << std::endl;

    return resulting_clouds;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr clstr::clustering::getCloudFromVector(std::vector<pcl::PointCloud<clstr::PointBool>::Ptr> clouds)
{
    std::cout << "The program will now colour each cluster with an unique color. " << std::flush;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointCloud<clstr::PointBool>::Ptr>::iterator vector_it;
    pcl::PointCloud<clstr::PointBool>::iterator clouds_it;
    for(vector_it=clouds.begin(); vector_it!=clouds.end(); vector_it++)
    {
        cloud_manip::convertBoolToXYZRGB(*vector_it, cloud_XYZRGB);
        //cloud_manip::giveRandomColorToCloud(cloud_XYZRGB);
        (*final_cloud)+=(*cloud_XYZRGB);
    }
    std::cout << "Finished attributing colours." << std::endl;
    return final_cloud;

}

//------------------------------------PRIVATE METHODS-------------------------------------------------------
void clstr::clustering::sortPointsByColor(pcl::PointCloud<clstr::PointBool>::iterator cloud_iterator)
{
    // Gets the point color and check if it already exists as a key in our <map>
    // If the key value has been found, we push the point into the cloud that corresponds to the key
    // Else we create a new cloud for a new key value and push the point in this new cloud instead
    std::string point_color = std::to_string(roundToNearestTenth((int)(*cloud_iterator).r)) + std::to_string(roundToNearestTenth((int)(*cloud_iterator).g)) + std::to_string(roundToNearestTenth((int)(*cloud_iterator).b));
    std::map<std::string, pcl::PointCloud<clstr::PointBool>::Ptr>::iterator map_it = color_map.find(point_color);
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

void clstr::clustering::createNewCluster(clstr::PointBool* crt_point, pcl::PointCloud<clstr::PointBool>::Ptr new_cluster)
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
            new_cluster->push_back(**nghbr_it);
            createNewCluster(*nghbr_it, new_cluster);
        }
    }
}

int clstr::clustering::roundToNearestTenth(int i)
{
    if(i%10<5)i=(i/10)*10;else{i=(i/10)*10+10;}return i;
}

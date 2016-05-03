/* Author : Kévin Naudin
 * Version : 1.0
 * Made for the I3 Mainz laboratory under GPL license
 */

#include "clustering.h"

//-----------------------------INITIALIZING STATIC ATTRIBUTES----------------------------------------------
std::map<std::string, pcl::PointCloud<clstr::point_clstr>::Ptr> clstr::clustering::color_map;
std::vector<pcl::PointCloud<clstr::point_clstr>::Ptr> clstr::clustering::resulting_clouds;

//------------------------------------PUBLIC METHODS-------------------------------------------------------

std::vector<pcl::PointCloud<clstr::point_clstr>::Ptr> clstr::clustering::getCloudsByColor(pcl::PointCloud<clstr::point_clstr>::Ptr base_cloud, double radius, size_t min_cluster_size)
{
    // Sorting the points by their RGB value
    pcl::PointCloud<clstr::point_clstr>::iterator cloud_iterator;
    std::cout << "Please wait while the algorithm sorts the cloud colours. " << std::flush;
    for(cloud_iterator = base_cloud->begin(); cloud_iterator!=base_cloud->end(); cloud_iterator++)
    {
        sortPointsByColor(cloud_iterator);
    }
    std::cout << "Approximated values and found " << color_map.size() << " different colours." << std::endl;

    // By iterating through the entire <map> we set a neighbourhood for each point of the currently looked at coloured cloud
    // By grouping every neighbourhood we will be able to segment the colour cloud into multiple segmented clouds of the same colour
    // We then check if the point has already been visited. If it's not the case, that means it must belong to another segmented cloud
    std::map<std::string, pcl::PointCloud<clstr::point_clstr>::Ptr>::iterator map_it;
    //These ints are only used for the informational text written on the console
    int colour_counter=0;

    std::cout << "The algorithm will now proceed to find the different clusters. This operation WILL take a while... " << std::flush;
    for(map_it = color_map.begin(); map_it!=color_map.end(); map_it++)
    {
        colour_counter++;
        if((map_it->second)->size() >= min_cluster_size)
        {
            std::cout << "Analyzing colour n°" << colour_counter << std::endl;
            setNeighbourhood(map_it->second, radius);
            pcl::PointCloud<clstr::point_clstr>::iterator cloud_it;
            for(cloud_it = (map_it->second)->begin(); cloud_it != (map_it->second)->end(); cloud_it++)
            {
                if(!(*cloud_it).getVisited()) // If the point hasn't been visited yet
                {
                    // We create a new cloud, set the point to visited then add him and its neighbours into the newly created cloud
                    pcl::PointCloud<clstr::point_clstr>::Ptr new_cluster (new pcl::PointCloud<clstr::point_clstr>);
                    (*cloud_it).setVisited(true);
                    new_cluster->push_back(*cloud_it);
                    createNewCluster(&(*cloud_it), new_cluster);
                    if(new_cluster->size()>=min_cluster_size)
                    {
                        resulting_clouds.push_back(new_cluster);
                    }
                    else { new_cluster->clear(); new_cluster->resize(0);}
                }
            }
        }
        // Frees memory by erasing the entry
        color_map.erase(map_it);
    }
    std::cout << " Finished finding clusters." << std::endl;

    return resulting_clouds;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr clstr::clustering::getCloudFromVector(std::vector<pcl::PointCloud<clstr::point_clstr>::Ptr> clouds)
{
    std::cout << "The program will now colour each cluster with an unique color. " << std::flush;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointCloud<clstr::point_clstr>::Ptr>::iterator vector_it;
    pcl::PointCloud<clstr::point_clstr>::iterator clouds_it;
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
void clstr::clustering::sortPointsByColor(pcl::PointCloud<clstr::point_clstr>::iterator cloud_iterator)
{
    // Gets the point color and check if it already exists as a key in our <map>
    // If the key value has been found, we push the point into the cloud that corresponds to the key
    // Else we create a new cloud for a new key value and push the point in this new cloud instead
    std::string point_color = std::to_string((*cloud_iterator).r) + std::to_string((*cloud_iterator).g) + std::to_string((*cloud_iterator).b);
    std::map<std::string, pcl::PointCloud<clstr::point_clstr>::Ptr>::iterator map_it = color_map.find(point_color);
    if(map_it != color_map.end()) // Means the key already exists
    {
        map_it->second->push_back(*cloud_iterator);
    }
    else
    {
        pcl::PointCloud<clstr::point_clstr>::Ptr new_colored_cloud (new pcl::PointCloud<clstr::point_clstr>);
        color_map[point_color] = new_colored_cloud;
        new_colored_cloud->push_back(*cloud_iterator);
    }
}

void clstr::clustering::setNeighbourhood(pcl::PointCloud<clstr::point_clstr>::Ptr colored_cloud, double search_radius)
{
    // Using a kdtree we are able to sort the points by coordinates
    // Since the clouds used are already sorted by colour, we are sure that each point find in a search radius is a true neighbour to another
    pcl::KdTreeFLANN<clstr::point_clstr> kdtree;
    kdtree.setInputCloud(colored_cloud);
    pcl::PointCloud<clstr::point_clstr>::iterator cloud_it;
    for(cloud_it = colored_cloud->begin(); cloud_it!=colored_cloud->end(); cloud_it++)
    {
        std::vector<int> PointsID; // Contains the ID of the neighbours
        std::vector<float> distances; // Contains the squared distances of the nieghbours (useless but mandatory)
        if(kdtree.radiusSearch(*cloud_it, search_radius, PointsID, distances) > 1) // Note that 0 refers to the point itself
        {
            for(size_t i = 0; i<PointsID.size(); i++)
            {
                if(!colored_cloud->points[PointsID[i]].getAdded())
                {
                    colored_cloud->points[PointsID[i]].setAdded(true);
                    (*cloud_it).addNeighbour(&(colored_cloud->points[PointsID[i]])); // We add a pointer of the neighbour because storing pointer costs way less rapid access memory than a copy of the neighbour itself
                }
            }
        }
        // Deleting a vector does not free memory so we clear it and shrink its size to what space it really needs. In this case, the space is of 0.
        PointsID.clear();
        PointsID.shrink_to_fit();
        distances.clear();
        distances.shrink_to_fit();
    }
}

void clstr::clustering::createNewCluster(clstr::point_clstr* crt_point, pcl::PointCloud<clstr::point_clstr>::Ptr new_cluster)
{
    for(clstr::point_clstr* point : crt_point->getNghbr())
    {
        if(!(point->getVisited()))
        {
            point->setVisited(true);
            new_cluster->push_back(*point);
            createNewCluster(point, new_cluster);
        }
    }
}

int clstr::clustering::roundToNearestTenth(int i)
{
    if(i%10<5)i=(i/10)*10;else{i=(i/10)*10+10;}return i;
}

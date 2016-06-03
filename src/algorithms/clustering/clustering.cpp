/* Author : Kévin Naudin
 * Version : 1.0
 * Made for the I3 Mainz laboratory under GPL license
 */

#include "clustering.h"

// Initializes static variables
std::map<std::string, pcl::PointCloud<clstr::point_clstr>::Ptr> clstr::clustering::coloured_clouds_map;
std::vector<clstr::point_clstr*> clstr::clustering::neighboursNotYetPushed;

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clstr::clustering::getClustersFromColouredCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double neighbours_radius, bool isWidop, size_t min_cluster_size)
{
    // If widop file, rescale to usable coordinates
    if(isWidop) cloud_object_segmentation::cloud_manip::scale_cloud(cloud, 1, 100, 1);

    // Vector storing the clusters
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> returned_clusters;

    // This algorithm works only with Cluster Points
    pcl::PointCloud<clstr::point_clstr>::Ptr base_cloud (new pcl::PointCloud<clstr::point_clstr>);
    cloud_object_segmentation::cloud_manip::convertXYZRGBToClstr(cloud, base_cloud);

    // Frees memory by deleting the unused cloud
    cloud->points.clear();
    cloud->points.shrink_to_fit();
    cloud = nullptr;

    // Creates as many fragmented cloud as there are colours
    clstr::clustering::sortPointsByColor(base_cloud);
    base_cloud->points.clear();
    base_cloud->points.shrink_to_fit();
    base_cloud = nullptr;

    std::cout << coloured_clouds_map.size() << " colours found." << std::endl;

    // We will go through the entire map to get each unicoloured cloud as there are and as many points as they contain
    std::map<std::string, pcl::PointCloud<clstr::point_clstr>::Ptr>::iterator map_iterator;
    pcl::PointCloud<clstr::point_clstr>::iterator cloud_iterator;
    int colour_counter = 0;
    int count_clusters = 0;

    std::vector<clstr::point_clstr*> neighboursToSee;
    std::vector<clstr::point_clstr*> useless_vector;
    std::vector<clstr::point_clstr*> resultingNeighbours;
    size_t i;

    for(map_iterator=coloured_clouds_map.begin(); map_iterator!=coloured_clouds_map.end(); map_iterator++)
    {
        colour_counter++;
        if(map_iterator->first != "000" &&  map_iterator->first !="00200")
        {
            std::cout << "Analizing coloured cloud number " << colour_counter << std::endl;
            std::cout << "Hint : This coloured cloud contains " << (map_iterator->second)->size() << " points" << std::endl;
            // As we need a kdtree to know our points neighbours but the vector it creates take a lot of memory, we will in advance tell our points who are their neighbours
            clstr::clustering::setNeighbourhoodForPoints(map_iterator->second, neighbours_radius);

            // Now we need to create the clusters for this unicoloured cloud
            for(cloud_iterator=(map_iterator->second)->begin(); cloud_iterator!=(map_iterator->second)->end(); cloud_iterator++)
            {
                if(!(*cloud_iterator).getVisited())
                {
                    // Deallocates memory used by the vector
                    neighboursToSee.clear();
                    neighboursToSee.shrink_to_fit();

                    pcl::PointCloud<clstr::point_clstr>::Ptr new_cluster (new pcl::PointCloud<clstr::point_clstr>);
                    new_cluster->push_back(*cloud_iterator);

                    neighboursToSee.push_back(&(*cloud_iterator));
                    i = 0;
                    while(i < neighboursToSee.size())
                    {
                        resultingNeighbours.clear();
                        resultingNeighbours.shrink_to_fit();
                        resultingNeighbours = clstr::clustering::addNeighboursIntoCluster(neighboursToSee[i], new_cluster);
                        std::merge(useless_vector.begin(), useless_vector.end(), resultingNeighbours.begin(), resultingNeighbours.end(), std::back_inserter(neighboursToSee));
                        i++;
                    }
                    if(new_cluster->size() >= min_cluster_size)
                    {
                        count_clusters++;
                        std::cout << "Cluster added because it contains " << new_cluster->size() << " points" << std::endl;
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
                        cloud_object_segmentation::cloud_manip::convertClstrToXYZRGB(new_cluster, cloud_xyzrgb);
                        new_cluster->points.clear();
                        new_cluster->points.shrink_to_fit();
                        new_cluster = nullptr;
                        cloud_object_segmentation::cloud_manip::scale_cloud(cloud_xyzrgb, 1, ((float)1/(float)100), 1);
                        returned_clusters.push_back(cloud_xyzrgb);
                    }
                    else
                    {
                        new_cluster->points.clear();
                        new_cluster->points.shrink_to_fit();
                        new_cluster = nullptr;
                    }
                }
            }
        }
        (map_iterator->second)->points.clear();
        (map_iterator->second)->points.shrink_to_fit();
        coloured_clouds_map.erase(map_iterator);
    }
    return returned_clusters;
}

void clstr::clustering::sortPointsByColor(pcl::PointCloud<clstr::point_clstr>::Ptr base_cloud)
{
    pcl::PointCloud<clstr::point_clstr>::iterator cloud_iterator;
    std::map<std::string, pcl::PointCloud<clstr::point_clstr>::Ptr>::iterator map_iterator;
    // Strings are less memory eating than floats
    std::string point_color;

    for(cloud_iterator=base_cloud->begin(); cloud_iterator!=base_cloud->end(); cloud_iterator++)
    {
        point_color = std::to_string((int)(*cloud_iterator).r) + std::to_string((int)(*cloud_iterator).g) + std::to_string((int)(*cloud_iterator).b);
        map_iterator=coloured_clouds_map.find(point_color);
        // If the colour has already been seen in the map we basically had the point to the pre-existing coloured cloud
        if(map_iterator != coloured_clouds_map.end())
        {
            (map_iterator->second)->push_back(*cloud_iterator);
        }
        // If the colour has never been seen we create a new cloud for this colour
        else
        {
            pcl::PointCloud<clstr::point_clstr>::Ptr unicoloured_cloud (new pcl::PointCloud<clstr::point_clstr>);
            unicoloured_cloud->push_back(*cloud_iterator);
            coloured_clouds_map[point_color] = unicoloured_cloud;
        }
    }
}

void clstr::clustering::setNeighbourhoodForPoints(pcl::PointCloud<clstr::point_clstr>::Ptr unicoloured_cloud, double neighbours_radius)
{
    pcl::PointCloud<clstr::point_clstr>::iterator cloud_iterator;

    pcl::KdTreeFLANN<clstr::point_clstr> kdtree;
    kdtree.setInputCloud(unicoloured_cloud);

    std::vector<int> PointsID; // Contains the neighbours indices
    std::vector<float> PointsDist; // Only needed by the KDTree for the radius search

    for(cloud_iterator=unicoloured_cloud->begin(); cloud_iterator!=unicoloured_cloud->end(); cloud_iterator++)
    {
        // Checks if the point has at least one neighbour except itself
        if(kdtree.radiusSearch(*cloud_iterator, neighbours_radius, PointsID, PointsDist) >= 1)
        {
            for(size_t i = 1; i<PointsID.size(); i++)
            {
                (*cloud_iterator).addNeighbour(&(unicoloured_cloud->points[PointsID[i]]));
            }
        }
        PointsID.clear();
        PointsID.shrink_to_fit();
        PointsDist.clear();
        PointsDist.shrink_to_fit();
    }
    kdtree = nullptr;
}

std::vector<clstr::point_clstr*> clstr::clustering::addNeighboursIntoCluster(clstr::point_clstr* point_ptr, pcl::PointCloud<clstr::point_clstr>::Ptr cluster)
{
    neighboursNotYetPushed.clear();
    neighboursNotYetPushed.shrink_to_fit();
    std::list<clstr::point_clstr*>::iterator nghbr_it;

    point_ptr->setVisited(true);
    for(nghbr_it=point_ptr->getIteratorOnFirstNeighbour(); nghbr_it!=point_ptr->getIteratorOnLastNeighbour(); nghbr_it++)
    {
        if(!(*nghbr_it)->getAdded())
        {
            (*nghbr_it)->setAdded(true);
            cluster->push_back(**nghbr_it);
            neighboursNotYetPushed.push_back(*nghbr_it);
        }
    }
    point_ptr->clearNeighbours();

    return neighboursNotYetPushed;
}

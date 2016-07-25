#include "../include/ModelDetection.h"

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/cloud_iterator.h>


void cos_lib::coloringProcess(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> inliers, std::vector<int> color){

    for(uint i=0; i<inliers.size(); i++){
        cloud->points[inliers[i]].r=color[0];
        cloud->points[inliers[i]].g=color[1];
        cloud->points[inliers[i]].b=color[2];
    }
}

void cos_lib::coloringPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> color){
    pcl::PointCloud<pcl::PointXYZRGB>::iterator it;

    for(it=cloud->begin(); it<cloud->end(); it++){
        (*it).r = color[0];
        (*it).g = color[1];
        (*it).b = color[2];
    }
 }

std::vector<int> cos_lib::getBestPlan(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,double distanceThreshold){

    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model);
    std::vector<int> inliers;
    ransac.setDistanceThreshold(distanceThreshold);
    ransac.computeModel();
    ransac.getInliers(inliers);

    return inliers;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cos_lib::getSubCloudFromIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr res (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(uint i=0; i<indices.size(); i++){
        res->points.push_back(cloud->at(indices[i]));
    }
    return res;
}

void cos_lib::colorPlans(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,double distanceThreshold,int pointsPerPlane){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored (new pcl::PointCloud<pcl::PointXYZRGB>);
    *temp = *cloud;
    colored->clear();

    int i = 0;
    while(temp->size()>(uint)pointsPerPlane){
        std::vector<int> inliers = getBestPlan(temp, distanceThreshold);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempPlane (new pcl::PointCloud<pcl::PointXYZRGB>);
        tempPlane = getSubCloudFromIndices(temp, inliers);
        temp = removeSetOfIndices(temp, inliers);

        if((int) inliers.size() >= pointsPerPlane){
            std::vector<int> color = colorRandomizer();
            coloringPointCloud(tempPlane, color);
            *colored += *tempPlane;
        }

        i++;
    }

    cloud->clear();
    *cloud = *colored;
}


std::vector<int> cos_lib::getBestLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,double distanceThreshold){

    pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model (new pcl::SampleConsensusModelLine<pcl::PointXYZRGB> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model);
    std::vector<int> inliers;
    ransac.setDistanceThreshold(distanceThreshold);
    ransac.computeModel();
    ransac.getInliers(inliers);

    return inliers;
}

void cos_lib::colorLines(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,double distanceThreshold,int pointsPerLine){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored (new pcl::PointCloud<pcl::PointXYZRGB>);
    *temp = *cloud;
    colored->clear();

    int i = 0;
    while(temp->size()>(uint)pointsPerLine){
        std::vector<int> inliers = getBestLine(temp, distanceThreshold);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempPlane (new pcl::PointCloud<pcl::PointXYZRGB>);
        tempPlane = getSubCloudFromIndices(temp, inliers);
        temp = removeSetOfIndices(temp, inliers);

        if((int) inliers.size() >= pointsPerLine){
            std::vector<int> color = colorRandomizer();
            coloringPointCloud(tempPlane, color);
            *colored += *tempPlane;
        }

        i++;
    }

    cloud->clear();
    *cloud = *colored;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cos_lib::removeSetOfIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::IndicesPtr indicesPtr (new std::vector<int>(indices));
    pcl::ExtractIndices<pcl::PointXYZRGB> filter;
    filter.setNegative(true);
    filter.setInputCloud(cloud);
    filter.setIndices(indicesPtr);
    filter.filter(*temp);

    return temp;
}

std::vector<int> cos_lib::colorRandomizer(){
    int r = rand()%256;
    int g = rand()%256;
    int b = rand()%256;

    std::vector<int> color;
    color.push_back(r);
    color.push_back(g);
    color.push_back(b);

    return color;
}

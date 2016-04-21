#include "lineFinding.h"

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/PointIndices.h>
#include <Eigen/StdVector>
#include <pcl/cloud_iterator.h>
#include <iterator>
#include <pcl/filters/extract_indices.h>
#include <stdlib.h>
#include <time.h>

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > lineFinding::lineColoring(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud){

    srand(time(NULL));

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > colored (new pcl::PointCloud<pcl::PointXYZRGB>);
    *temp = *cloud;
    colored->clear();

    int i = 0;
    while( i<10 && temp->size()>1000){
        std::vector<int> inliers = findBestPlane(temp);
        std::vector<int> color = colorRandomizer();
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > tempPlane (new pcl::PointCloud<pcl::PointXYZRGB>);

        tempPlane = findOnePlane(temp, inliers);
        temp = removeSetOfIndices(temp, inliers);

        std::cout << "step #" <<i+1<< " | # of points in temp:"<<temp->size() << std::endl;
        std::cout << "# of inliers:" << inliers.size() << std::endl;

        if(inliers.size()>1000){
            colorEntirePlane(tempPlane, color);
            *colored += *tempPlane;
        }


        i++;
    }

    return colored;
}

void lineFinding::coloringOneLine(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> inliers, std::vector<int> color){

    for(int i=0; i<inliers.size(); i++){
        cloud->points[inliers[i]].r=color[0];
        cloud->points[inliers[i]].g=color[1];
        cloud->points[inliers[i]].b=color[2];
    }
}


std::vector<int> lineFinding::findBestPlane(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud){

    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model);
    std::vector<int> inliers;
    ransac.setDistanceThreshold(0.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    return inliers;
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > lineFinding::removeSetOfIndices(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> indices){

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::IndicesPtr indicesPtr (new std::vector<int>(indices));
    pcl::ExtractIndices<pcl::PointXYZRGB> filter;
    filter.setNegative(true);
    filter.setInputCloud(cloud);
    filter.setIndices(indicesPtr);
    filter.filter(*temp);

    return temp;
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > lineFinding::findOnePlane(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> indices){

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > res (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0; i<indices.size(); i++){
        res->points.push_back(cloud->at(indices[i]));
    }
    return res;
}

void lineFinding::colorEntirePlane(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> color){
    pcl::PointCloud<pcl::PointXYZRGB>::iterator it;

    for(it=cloud->begin(); it<cloud->end(); it++){
        (*it).r = color[0];
        (*it).g = color[1];
        (*it).b = color[2];
    }
}

std::vector<int> lineFinding::colorRandomizer(){

    int r = rand()%256;
    int g = rand()%256;
    int b = rand()%256;

    std::vector<int> color;
    color.push_back(r);
    color.push_back(g);
    color.push_back(b);

    std::cout <<"r:"<<color[0]<<" g:"<<color[1]<<" b:"<<color[2]<< std::endl;
    return color;

}

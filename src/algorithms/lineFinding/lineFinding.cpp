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

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    *temp = *cloud;


    for(int i=0; i<10; i++){
        std::vector<int> inliers = findBestLine(temp);
        temp = removeSetOfIndices(temp, inliers);

        std::vector<int> color = colorRandomizer();

        coloringOneLine(cloud, inliers, color);
    }


    return cloud;
}

void lineFinding::coloringOneLine(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> inliers, std::vector<int> color){

    for(int i=0; i<inliers.size(); i++){
        cloud->points[inliers[i]].r=color[0];
        cloud->points[inliers[i]].g=color[1];
        cloud->points[inliers[i]].b=color[2];
    }
}


std::vector<int> lineFinding::findBestLine(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud){

    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model);
    std::vector<int> inliers;
    ransac.setDistanceThreshold(.02);
    ransac.computeModel();
    ransac.getInliers(inliers);
    /*
    Eigen::VectorXf coef;
    Eigen::VectorXf coefRefined;
    ransac.getModelCoefficients(coef);
    model->optimizeModelCoefficients(inliers, coef, coefRefined);
    model->selectWithinDistance(coefRefined, .01, inliers);*/

    //std::cout << coefRefined << std::endl;
    std::cout << inliers.size() << std::endl;

    return inliers;
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > lineFinding::removeSetOfIndices(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> indices){

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::IndicesPtr indicesPtr (new std::vector<int>(indices));
    std::cout << "test : " << cloud->size() << " | " << indicesPtr->size() << std::endl;
    pcl::ExtractIndices<pcl::PointXYZRGB> filter;
    filter.setNegative(true);
    filter.setInputCloud(cloud);
    filter.setIndices(indicesPtr);
    filter.filter(*temp);

    std::cout << temp->size() << std::endl;

    return temp;
}

std::vector<int> lineFinding::colorRandomizer(){
    srand(time(NULL));
    int r = rand()%256;
    int g = rand()%256;
    int b = rand()%256;

    std::vector<int> color;
    color.push_back(b);
    color.push_back(g);
    color.push_back(r);

    return color;

}

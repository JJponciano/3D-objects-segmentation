#include "lineFinding.h"

#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <Eigen/StdVector>
#include <pcl/cloud_iterator.h>
#include <iterator>
#include <pcl/filters/extract_indices.h>
#include <stdlib.h>
#include <time.h>
#include <pcl/filters/morphological_filter.h>
#include <cstddef>
#include <pcl/sample_consensus/rransac.h>

#include "plane.h"
#include "line.h"

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > lineFinding::lineColoring(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud){

    srand(time(NULL));

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > colored (new pcl::PointCloud<pcl::PointXYZRGB>);
    *temp = *cloud;
    colored->clear();

    int i = 0;
    while( i<10 && temp->size()>1000){
        std::vector<int> inliers;
        Eigen::VectorXf coef;
        Plane* p = findBestPlane(temp, inliers, coef);
        inliers = p->getInliers();
        coef = p->getCoefficients();
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

std::vector<float> lineFinding::minMaxCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud){
    float minX, maxX, minY, maxY, minZ, maxZ;
    pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
    it=cloud->begin();
    minX = (*it).x;
    maxX = (*it).x;
    minY = (*it).y;
    maxY = (*it).y;
    minZ = (*it).z;
    maxZ = (*it).z;

    for(it=cloud->begin(); it<cloud->end(); it++){
        if((*it).x < minX){
            minX = (*it).x;
        }else{
            if((*it).x > maxX){
                maxX = (*it).x;
            }
        }

        if((*it).y < minY){
            minY = (*it).y;
        }else{
            if((*it).y > maxY){
                maxY = (*it).y;
            }
        }

        if((*it).z < minZ){
            minZ = (*it).z;
        }else{
            if((*it).z > maxZ){
                maxZ = (*it).z;
            }
        }
    }

    std::cout << "minx:" << minX << " maxx:" << maxX << std::endl;
    std::cout << "miny:" << minY << " maxy:" << maxY << std::endl;
    std::cout << "minz:" << minZ << " maxz:" << maxZ << std::endl;

    std::vector<float> res;
    res.push_back(minX);
    res.push_back(maxX);
    res.push_back(minY);
    res.push_back(maxY);
    res.push_back(minZ);
    res.push_back(maxZ);
    return res;
}

Plane*  lineFinding::findBestPlane(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> inliers, Eigen::VectorXf coef){

    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model);
    ransac.setDistanceThreshold(0.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
    ransac.getModelCoefficients(coef);

    Plane* p = new Plane(inliers, coef);


    std::cout << "plane coef: " << coef[0] << " | " << coef[1] << " | " << coef[2] << " | " << coef[3] << " | " << std::endl;

    return p;
}

float lineFinding::avgDistanceBetweenPoints(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud){
    pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
    std::vector<float> avgs;
    float avgDistance;
    int K = 5;

    for(int j =0; j<10; j++){
        avgDistance = 0;
        int alea = (int) rand()%cloud->size()+1;
        it = cloud->begin();
        for(int i=0; i<alea; i++) it++;

        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        kdtree.setInputCloud(cloud);
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if ( kdtree.nearestKSearch ((*it), K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
                avgDistance+= sqrt(pointNKNSquaredDistance[i]);
            }
            avgDistance = avgDistance/K;
        }
        avgs.push_back(avgDistance);
    }

    float temp=0;
    for(int i =0; i<avgs.size(); i++){
        temp += avgs[i];
    }
    return (temp/avgs.size());

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
    for(uint i=0; i<indices.size(); i++){
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

void lineFinding::coloringOneLine(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> inliers, std::vector<int> color){

    for(uint i=0; i<inliers.size(); i++){
        cloud->points[inliers[i]].r=color[0];
        cloud->points[inliers[i]].g=color[1];
        cloud->points[inliers[i]].b=color[2];
    }
}

void lineFinding::findLinesInYDirection(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud){
    srand(time(NULL));

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > colored (new pcl::PointCloud<pcl::PointXYZRGB>);
    *temp = *cloud;
    colored->clear();

    int i = 0;
    while( i<10 && temp->size()>1000){
        std::vector<int> inliers;
        Eigen::VectorXf coef;
        Line* l = findALineInYDirection(temp);
        if(l!=NULL){
            inliers = l->getInliers();
            coef = l->getCoefficients();
            std::vector<int> color = colorRandomizer();
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > tempLine (new pcl::PointCloud<pcl::PointXYZRGB>);

            tempLine = findOnePlane(temp, inliers);
            temp = removeSetOfIndices(temp, inliers);

            std::cout << "step #" <<i+1<< " | # of points in temp:"<<temp->size() << std::endl;
            std::cout << "# of inliers:" << inliers.size() << std::endl;

            if(inliers.size()>0){
                colorEntirePlane(tempLine, color);
                *colored += *tempLine;

            }
        }
        i++;
    }

    cloud->clear();
    *cloud = *colored;
}

Line* lineFinding::findALineInYDirection(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud){

    pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model (new pcl::SampleConsensusModelLine<pcl::PointXYZRGB> (cloud));
    //pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model);
    pcl::RandomizedRandomSampleConsensus<pcl::PointXYZRGB> ransac (model);
    std::vector<int> inliers;
    Eigen::VectorXf coefficients;
    ransac.setDistanceThreshold(0.01);
    Line* l = NULL;
    bool exit = false;
    int i = 0;
    while(l == NULL & !exit){
        ransac.computeModel();
        ransac.getInliers(inliers);
        ransac.getModelCoefficients(coefficients);

        float x = fabs(coefficients[3]);
        float y = fabs(coefficients[4]);
        float z = fabs(coefficients[5]);
        i++;
        if((y-x)>0 && (y-z)>0){
            l = new Line(inliers, coefficients);
            exit = true;
        }else{
            if(i >= 8){
                exit = true;
            }
        }
    }




    return l;
}

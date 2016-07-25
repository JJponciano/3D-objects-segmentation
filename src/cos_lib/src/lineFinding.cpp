#include "../include/lineFinding.h"

#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <Eigen/StdVector>
#include <pcl/cloud_iterator.h>
#include <iterator>
#include <pcl/filters/extract_indices.h>
#include <stdlib.h>
#include <time.h>
#include <pcl/sample_consensus/rransac.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cos_lib::lineColoring(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    srand(time(NULL));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored (new pcl::PointCloud<pcl::PointXYZRGB>);
    *temp = *cloud;
    colored->clear();

    int i = 0;
    while( i<1 && temp->size()>1000){
        std::vector<int> inliers;
        Eigen::VectorXf coef;
        cos_lib::Plane* p = findBestPlane(temp, inliers, coef);
        inliers = p->getInliers();
        coef = p->getCoefficients();
        std::vector<int> color = colorRandomizer();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempPlane (new pcl::PointCloud<pcl::PointXYZRGB>);

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

cos_lib::Plane*  cos_lib::findBestPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> inliers, Eigen::VectorXf coef){

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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cos_lib::findOnePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr res (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(uint i=0; i<indices.size(); i++){
        res->points.push_back(cloud->at(indices[i]));
    }
    return res;
}

void cos_lib::colorEntirePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> color){
    pcl::PointCloud<pcl::PointXYZRGB>::iterator it;

    for(it=cloud->begin(); it<cloud->end(); it++){
        (*it).r = color[0];
        (*it).g = color[1];
        (*it).b = color[2];
    }
}

void cos_lib::coloringOneLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> inliers, std::vector<int> color){

    for(uint i=0; i<inliers.size(); i++){
        cloud->points[inliers[i]].r=color[0];
        cloud->points[inliers[i]].g=color[1];
        cloud->points[inliers[i]].b=color[2];
    }
}

void cos_lib::findLinesInYDirection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    srand(time(NULL));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored (new pcl::PointCloud<pcl::PointXYZRGB>);
    *temp = *cloud;
    colored->clear();

    int i = 0;
    while( i<10 && temp->size()>1000){
        std::vector<int> inliers;
        Eigen::VectorXf coef;
        Line* l = findALineInYDirection(temp);
        if(l!=NULL){
            inliers = l->getInliers();
            coef = (*l->getCoefficients());
            l->getAnglesToOrigin();
            std::vector<int> color = colorRandomizer();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempLine (new pcl::PointCloud<pcl::PointXYZRGB>);

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

cos_lib::Line* cos_lib::findALineInYDirection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model (new pcl::SampleConsensusModelLine<pcl::PointXYZRGB> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model);
    //pcl::RandomizedRandomSampleConsensus<pcl::PointXYZRGB> ransac (model);
    std::vector<int> inliers;
    Eigen::VectorXf coefficients;
    ransac.setDistanceThreshold(0.01);
    Line* l = NULL;
    bool exit = false;
    int i = 0;
    while(!exit){
        ransac.computeModel();
        ransac.getInliers(inliers);
        ransac.getModelCoefficients(coefficients);

        float x = fabs(coefficients[3]);
        float y = fabs(coefficients[4]);
        float z = fabs(coefficients[5]);
        i++;
        if((y-x)>0 && (y-z)>0){
            l = new Line(inliers, &coefficients);
            exit = true;
        }else{
            if(i >= 8){
                exit = true;
            }
        }
    }

    return l;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr cos_lib::findLinesInClusters(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters){
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it = clusters.begin();
    std::vector<Line*> lines;
    std::vector<Line*> temp;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr res (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    for(it = clusters.begin(); it != clusters.end(); it++){

        temp2 = findLines(*it);
        *res += *temp2;
//        temp = findLines(*it);

        lines.insert(lines.end(), temp.begin(), temp.end());
    }

    return res;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cos_lib::findLines(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    srand(time(NULL));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<Line*> lines;
    *temp = *cloud;

    int i = 0;
    while( i<25 && temp->size()>1000){
        std::vector<int> inliers;
        Eigen::VectorXf* coef;
        Line* l = findBestLine(temp);
        if(l!=NULL){
            inliers = l->getInliers();
            coef = l->getCoefficients();
            Line* ltemp = new Line(l->getInliers(),l->getCoefficients());


            lines.push_back(ltemp);

            std::vector<int> color = colorRandomizer();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempPlane (new pcl::PointCloud<pcl::PointXYZRGB>);

            tempPlane = findOnePlane(temp, inliers);

            colorEntirePlane(tempPlane, color);
            *colored += *tempPlane;

            temp = removeSetOfIndices(temp, inliers);

        }
        i++;
    }

    cloud->clear();
//    return lines;
    return colored;
}


cos_lib::Line* cos_lib::findBestLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model (new pcl::SampleConsensusModelLine<pcl::PointXYZRGB> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model);
    std::vector<int> inliers;
    Eigen::VectorXf coefficients;
    ransac.setDistanceThreshold(0.01);
    Line* l = NULL;
    ransac.computeModel();
    ransac.getInliers(inliers);
    ransac.getModelCoefficients(coefficients);
    l = new Line(inliers, &coefficients);

    return l;
}

std::vector<float> cos_lib::findIntersections(std::vector<Line*> lines){

    int length = lines.size();
    int i, j;
    std::vector<float> angles;
    Line* l = new Line();
    float angle;

    for(i = 0; i<length-1; i++){
        for(j = i; j<length; j++){
            angle = l->angleBetweenLines(lines.at(i), lines.at(j));
            if(angle != 0) angles.push_back(angle);
        }
    }

    return angles;
}

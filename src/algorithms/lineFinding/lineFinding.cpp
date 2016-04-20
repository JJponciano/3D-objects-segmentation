#include "lineFinding.h"

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > lineFinding::lineColoring(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(cloud, temp);
    pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model (new pcl::SampleConsensusModelLine<pcl::PointXYZRGB> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model);
    std::vector<int> inliers;
    ransac.setDistanceThreshold(.02);
    ransac.computeModel();
    ransac.getInliers(inliers);

    Eigen::VectorXf coef;
    Eigen::VectorXf coefRefined;
    ransac.getModelCoefficients(coef);
    model->optimizeModelCoefficients(inliers, coef, coefRefined);
    model->selectWithinDistance(coefRefined, .01, inliers);

    std::vector<int> color;
    color.push_back(255);
    color.push_back(0);
    color.push_back(0);

    coloringOneLine(cloud, inliers, color);

    std::cout << coefRefined << std::endl;
    std::cout << inliers.size() << std::endl;

    return cloud;
}

void lineFinding::coloringOneLine(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, std::vector<int> inliers, std::vector<int> color){

    for(int i=0; i<inliers.size(); i++){
        cloud->points[inliers[i]].r=color[0];
        cloud->points[inliers[i]].g=color[1];
        cloud->points[inliers[i]].b=color[2];
    }
}

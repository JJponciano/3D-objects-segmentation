#include "../include/line.h"
#include <stdio.h>
#include <iostream>
#include <pcl/common/intersections.h>

cos_lib::Line::Line(){

}

cos_lib::Line::Line(std::vector<int> inliers, Eigen::VectorXf* coefficients)
{
    this->inliers = inliers;
    this->coefficients = coefficients;
}

std::vector<int> cos_lib::Line::getInliers(){
    return this->inliers;
}

Eigen::VectorXf *cos_lib::Line::getCoefficients(){
    return this->coefficients;
}

std::vector<float> cos_lib::Line::getAnglesToOrigin(){

    std::vector<float> res;
    Eigen::Vector3f x(1,0,0);
    Eigen::Vector3f y(0,1,0);
    Eigen::Vector3f z(0,0,1);

    Eigen::Vector3f direction;
    direction[0] = (*coefficients)[3]; direction[1] = (*coefficients)[4];
    direction [2] = (*coefficients)[5];

    std::cout << x.dot(y) << std::endl;
    float angleX = acos((x.dot(direction)));
    float angleY = acos((y.dot(direction)));
    float angleZ = acos((z.dot(direction)));
    res.push_back(angleX);
    res.push_back(angleY);
    res.push_back(angleZ);

    return res;
}

float cos_lib::Line::angleBetweenLines(Line* l1, Line* l2){
    Eigen::Vector4f point;

    Eigen::VectorXf* coefficients1 = l1->getCoefficients();

    std::cout << (*coefficients1)[3] << std::endl;
    Eigen::Vector3f direction1((*coefficients1)[3], (*coefficients1)[4], (*coefficients1)[5]);

    Eigen::VectorXf* coefficients2 = l2->getCoefficients();
    Eigen::Vector3f direction2((*coefficients2)[3], (*coefficients2)[4], (*coefficients2)[5]);

    float angle = 0;
    if(pcl::lineWithLineIntersection((*coefficients1), (*coefficients2), point)){
         angle = acos((direction1.dot(direction2)));
    }

    return angle;
}

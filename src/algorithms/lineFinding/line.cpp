#include "line.h"
#include <stdio.h>
#include <iostream>

Line::Line(std::vector<int> inliers, Eigen::VectorXf coefficients)
{
    this->inliers = inliers;
    this->coefficients = coefficients;
}

std::vector<int> Line::getInliers(){
    return this->inliers;
}

Eigen::VectorXf Line::getCoefficients(){
    return this->coefficients;
}

void Line::toConsole(){
    std::cout << "# of inliers : " << inliers.size() << std::endl;
    std::cout << "x: " << coefficients[3]  << " | y:" << coefficients[4] << " | z:" << coefficients[5]   << std::endl;
}

std::vector<float> Line::getAnglesToOrigin(){

    std::vector<float> res;
    Eigen::VectorXf x;
    x << 1, 0, 0 ;
    Eigen::VectorXf y;
    y << 0, 1, 0 ;
    Eigen::VectorXf z;
    z << 0, 0, 1 ;

    Eigen::VectorXf direction;
    direction << coefficients[3], coefficients[4], coefficients[5];

    std::cout << x.dot(y) << std::endl;
    float angleX = acos((x.dot(direction)));
    float angleY = acos((y.dot(direction)));
    float angleZ = acos((z.dot(direction)));
    res.push_back(angleX);
    res.push_back(angleY);
    res.push_back(angleZ);

    return res;
}

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

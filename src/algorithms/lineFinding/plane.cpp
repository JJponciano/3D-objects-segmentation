#include "plane.h"

Plane::Plane(std::vector<int> inliers, Eigen::VectorXf coefficients)
{
    this->coefficients = coefficients;
    this->inliers = inliers;
}

std::vector<int> Plane::getInliers(){
    return this->inliers;
}

Eigen::VectorXf Plane::getCoefficients(){
    return this->coefficients;
}

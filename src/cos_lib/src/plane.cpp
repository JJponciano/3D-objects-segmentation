#include "../include/plane.h"

cos_lib::Plane::Plane(std::vector<int> inliers, Eigen::VectorXf coefficients)
{
    this->coefficients = coefficients;
    this->inliers = inliers;
}

std::vector<int> cos_lib::Plane::getInliers(){
    return this->inliers;
}

Eigen::VectorXf cos_lib::Plane::getCoefficients(){
    return this->coefficients;
}

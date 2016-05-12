#ifndef PLANE_H
#define PLANE_H

#include <vector>
#include <Eigen/StdVector>
class Plane
{
private:
       std::vector<int> inliers;
       Eigen::VectorXf coefficients;
public:
    Plane(std::vector<int> inliers, Eigen::VectorXf coefficients);
    std::vector<int> getInliers();
    Eigen::VectorXf getCoefficients();
};

#endif // PLANE_H

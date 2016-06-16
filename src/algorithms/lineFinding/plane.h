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
    /**
     * @brief Plane basic initialization constructor for planes in a point cloud
     * @param inliers the indices of the inliers of the plane
     * @param coefficients the coefficients that define the plane
     */
    Plane(std::vector<int> inliers, Eigen::VectorXf coefficients);

    /**
     * @brief getInliers returns the indices of the inliers
     * @return vector<int>
     */
    std::vector<int> getInliers();
    /**
     * @brief getCoefficients returns the coefficients that define the plane
     * @return Eigen::vectorXf
     */
    Eigen::VectorXf getCoefficients();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // PLANE_H

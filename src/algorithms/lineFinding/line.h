#ifndef LINE_H
#define LINE_H

#include <vector>
#include <Eigen/StdVector>
#include <QString>
#include <vector>

class Line
{
private:
    std::vector<int> inliers;
    Eigen::VectorXf coefficients;

public:
    Line(std::vector<int> inliers, Eigen::VectorXf coefficients);
    std::vector<int> getInliers();
    Eigen::VectorXf getCoefficients();
    void toConsole();
    std::vector<float> getAnglesToOrigin();
};

#endif // LINE_H

#ifndef LINE_H
#define LINE_H

#include <vector>
#include <Eigen/StdVector>
#include <QString>
#include <vector>

namespace cos_lib
{
    class Line
    {
    private:
        std::vector<int> inliers;
        Eigen::VectorXf* coefficients;

    public:
        /**
         * @brief Line
         */
        Line();

        /**
         * @brief Line basic initialization constructor
         * @param inliers the indices of the inliers of the line
         * @param coefficients the coefficients that define the line
         */
        Line(std::vector<int> inliers, Eigen::VectorXf* coefficients);

        /**
         * @brief getInliers returns the indices of the inliers
         * @return vector<int>
         */
        std::vector<int> getInliers();

        /**
         * @brief getCoefficients
         * @return Eigen::vectorXf
         */
        Eigen::VectorXf* getCoefficients();

        /**
         * @brief getAnglesToOrigin WIP not working, just used for testing
         * @return vector<float>
         */
        std::vector<float> getAnglesToOrigin();

        /**
         * @brief angleBetweenLines WIP, not working, just for testing
         * @param l1 1 line
         * @param l2 2nd line
         * @return float = angle
         */
        float angleBetweenLines(Line* l1, Line* l2);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif // LINE_H

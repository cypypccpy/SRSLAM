#include <Eigen/Core>
#include <Eigen/Geometry>

struct mappoint
{
    Eigen::Matrix<double, 3, 1> pos_ = Eigen::Matrix<double, 3, 1>::Zero();
};

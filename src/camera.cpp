#include "srslam/camera.h"

camera::camera() {};

Eigen::Matrix<double, 3, 1> camera::world2camera(const Eigen::Matrix<double, 3, 1> &p_w, const Eigen::Isometry3d &T_c_w) {
    return pose_ * T_c_w * p_w;
}

Eigen::Matrix<double, 3, 1> camera::camera2world(const Eigen::Matrix<double, 3, 1> &p_c, const Eigen::Isometry3d &T_c_w) {
    return T_c_w.inverse() * pose_inv_ * p_c;
}

Eigen::Matrix<double, 2, 1> camera::camera2pixel(const Eigen::Matrix<double, 3, 1> &p_c) {
    return Eigen::Matrix<double, 2, 1>(
            fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
            fy_ * p_c(1, 0) / p_c(2, 0) + cy_
    );
}

Eigen::Matrix<double, 3, 1> camera::pixel2camera(const Eigen::Matrix<double, 2, 1> &p_p, double depth) {
    return Eigen::Matrix<double, 3, 1>(
            (p_p(0, 0) - cx_) * depth / fx_,
            (p_p(1, 0) - cy_) * depth / fy_,
            depth
    );
}

Eigen::Matrix<double, 2, 1> camera::world2pixel(const Eigen::Matrix<double, 3, 1> &p_w, const Eigen::Isometry3d &T_c_w) {
    return camera2pixel(world2camera(p_w, T_c_w));
}

Eigen::Matrix<double, 3, 1> camera::pixel2world(const Eigen::Matrix<double, 2, 1> &p_p, const Eigen::Isometry3d &T_c_w, double depth) {
    return camera2world(pixel2camera(p_p, depth), T_c_w);
}

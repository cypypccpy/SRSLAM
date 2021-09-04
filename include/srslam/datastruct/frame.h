#pragma once

#include "srslam/camera.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <thread>
#include <mutex>
#include <unistd.h>

// forward declare
class mappoint;
class feature;

/**
 * 帧
 * 每一帧分配独立id，关键帧分配关键帧ID
 */
class frame {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    unsigned long id_ = 0;           // id of this frame
    unsigned long keyframe_id_ = 0;  // id of key frame
    bool is_keyframe_ = false;       // 是否为关键帧
    double time_stamp_;              // 时间戳，暂不使用
    Eigen::Isometry3d pose_;                       // Tcw 形式Pose
    std::mutex pose_mutex_;          // Pose数据锁
    cv::Mat left_img_, right_img_;   // stereo images

    // extracted features in left image
    std::vector<std::shared_ptr<feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<feature>> features_right_;

   public:  // data members
    frame() {}

    frame(long id, double time_stamp, const Eigen::Isometry3d &pose, const cv::Mat &left,
          const cv::Mat &right);

    // set and get pose, thread safe
    Eigen::Isometry3d Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const Eigen::Isometry3d &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    /// 设置关键帧并分配并键帧id
    void SetKeyFrame();

    /// 工厂构建模式，分配id 
    static std::shared_ptr<frame> CreateFrame();
};


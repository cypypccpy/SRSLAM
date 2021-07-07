#include <opencv2/features2d.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <srslam/mappoint.h>

struct frame {
    long id_;
    cv::Mat left_img_;
    cv::Mat right_img_;
    std::vector<cv::KeyPoint> left_keypoint_position_;
    std::vector<cv::KeyPoint> right_keypoint_position_;
    std::vector<mappoint> left_mappoint;
    std::vector<mappoint> right_mappoint;

    Sophus::SE3d pose_;              // Tcw 形式Pose
    double time_stamp_;
    
    std::mutex pose_mutex_;          // Pose数据锁

    frame() {};

    frame(long id, double time_stamp, const Sophus::SE3d &pose, const cv::Mat &left,
          const cv::Mat &right);

    // set and get pose, thread safe
    Sophus::SE3d Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const Sophus::SE3d &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }
};
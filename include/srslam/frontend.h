#pragma once

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <srslam/camera.h>
#include<opencv2/core/eigen.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <camera_info_manager/camera_info_manager.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>

enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

/**
 * 前端
 * 估计当前帧Pose，在满足关键帧条件时向地图加入关键帧并触发优化
 */
class Frontend {
    public:
        Frontend();

        /// 外部接口
        void SetCameras(std::shared_ptr<camera> left, std::shared_ptr<camera> right) {
            camera_left_ = left;
            camera_right_ = right;
        }

        void RegisterCallBack(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight);

    private:
        /**
         * Track in normal mode
         * @return true if success
         */
        bool Track();

        /**
         * Reset when lost
         * @return true if success
         */
        bool Reset();

        /**
         * Track with last frame
         * @return num of tracked points
         */
        int TrackLastFrame();

        /**
         * estimate current frame's pose
         * @return num of inliers
         */
        int EstimateCurrentPose();

        /**
         * set current frame as a keyframe and insert it into backend
         * @return true if success
         */
        bool InsertKeyframe();

        /**
         * Try init the frontend with stereo images saved in current_frame_
         * @return true if success
         */
        bool StereoInit();

        /**
         * Detect features in left image in current_frame_
         * keypoints will be saved in current_frame_
         * @return
         */
        int DetectFeatures();

        /**
         * Find the corresponding features in right image of current_frame_
         * @return num of features found
         */
        int FindFeaturesInRight();

        /**
         * Build the initial map with single image
         * @return true if succeed
         */
        bool BuildInitMap();

        /**
         * Triangulate the 2D points in current frame
         * @return num of triangulated points
         */
        int TriangulateNewPoints();

        /**
         * Set the features in keyframe as new observation of the map points
         */
        void SetObservationsForKeyFrame();

        // data
        Eigen::Isometry3d relative_motion_;  // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

        int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

        cv::Mat current_imgL_, current_imgR_, last_imgL_, last_imgR_;

        std::vector<cv::KeyPoint> current_left_keypoint_position_, current_right_keypoint_position_;

        std::shared_ptr<camera> camera_left_ = nullptr;   // 左侧相机
        std::shared_ptr<camera> camera_right_ = nullptr;  // 右侧相机
        FrontendStatus status_ = FrontendStatus::INITING;

        // params
        int num_features_ = 200;
        int num_features_init_ = 100;
        int num_features_tracking_ = 50;
        int num_features_tracking_bad_ = 20;
        int num_features_needed_for_keyframe_ = 80;

        // utilities
        cv::Ptr<cv::GFTTDetector> gftt_;  // feature detector in opencv

        //ros
        ros::NodeHandle nh_;  
        image_transport::ImageTransport* it_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> sync_pol;
        message_filters::Subscriber<sensor_msgs::Image>* right_sub_ ;
        message_filters::Subscriber<sensor_msgs::Image>* left_sub_;

        message_filters::Synchronizer<sync_pol>* sync_;
};

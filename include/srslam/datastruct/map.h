#pragma once
#ifndef MAP_H
#define MAP_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>

#include "srslam/datastruct/frame.h"
#include "srslam/datastruct/mappoint.h"
/**
 * @brief 地图
 * 和地图的交互：前端调用InsertKeyframe和InsertMapPoint插入新帧和地图点，后端维护地图的结构，判定outlier/剔除等等
 */
class map {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::unordered_map<unsigned long, std::shared_ptr<mappoint>> LandmarksType;
    typedef std::unordered_map<unsigned long, std::shared_ptr<frame>> KeyframesType;

    map() {}

    /// 增加一个关键帧
    void InsertKeyFrame(std::shared_ptr<frame> frame);
    /// 增加一个地图顶点
    void InsertMapPoint(std::shared_ptr<mappoint> map_point);

    /// 获取所有地图点
    LandmarksType GetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }
    /// 获取所有关键帧
    KeyframesType GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    /// 获取激活地图点
    LandmarksType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    /// 获取激活关键帧
    KeyframesType GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    /// 清理map中观测数量为零的点
    void CleanMap();

   private:
    // 将旧的关键帧置为不活跃状态
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;         // all landmarks
    LandmarksType active_landmarks_;  // active landmarks
    KeyframesType keyframes_;         // all key-frames
    KeyframesType active_keyframes_;  // all key-frames

    std::shared_ptr<frame> current_frame_ = nullptr;

    // settings
    int num_active_keyframes_ = 7;  // 激活的关键帧数量
};

#endif  // MAP_H

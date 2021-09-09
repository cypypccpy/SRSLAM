#pragma once

#include <memory>
#include <mutex>
#include <list>
#include <Eigen/Core>
#include <Eigen/Geometry>

class frame;

class feature;

/**
 * 路标点类
 * 特征点在三角化之后形成路标点
 */
class mappoint {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    unsigned long id_ = 0;  // ID
    bool is_outlier_ = false;
    Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();  // Position in world
    std::mutex data_mutex_;
    int observed_times_ = 0;  // being observed by feature matching algo.
    std::list<std::weak_ptr<feature>> observations_;
                
    mappoint() {}

    mappoint(long id, Eigen::Vector3d position);

    Eigen::Vector3d Pos() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return pos_;
    }

    void SetPos(const Eigen::Vector3d &pos) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        pos_ = pos;
    };

    void AddObservation(std::shared_ptr<feature> feature) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        observations_.push_back(feature);
        observed_times_++;
    }

    void RemoveObservation(std::shared_ptr<feature> feat);

    std::list<std::weak_ptr<feature>> GetObs() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return observations_;
    }

    // factory function
    static std::shared_ptr<mappoint> CreateNewMappoint();
};


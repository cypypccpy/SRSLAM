#include "srslam/backend.h"
#include "srslam/datastruct/feature.h"
#include "srslam/datastruct/map.h"
#include "srslam/datastruct/mappoint.h"

Backend::Backend() {
    backend_running_.store(true);
    backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
}

void Backend::UpdateMap() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void Backend::Stop() {
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();
}

void Backend::BackendLoop() {
    while (backend_running_.load()) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);

        /// 后端仅优化激活的Frames和Landmarks
        map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
        map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
        Optimize(active_kfs, active_landmarks);
    }
}

void Backend::Optimize(map::KeyframesType &keyframes,
                       map::LandmarksType &landmarks) {
    gtsam::Values initial_estimate;
    gtsam::NonlinearFactorGraph graph;
    const auto model = gtsam::noiseModel::Isotropic::Sigma(3, 1);
    // create stereo camera calibration object with .2m between cameras
    const gtsam::Cal3_S2Stereo::shared_ptr K(
        new gtsam::Cal3_S2Stereo(cam_left_->fx_, cam_left_->fy_, cam_left_->cx_,
                                    cam_left_->cy_, 1, cam_left_->baseline_));

    // camera and landmark keys
    std::vector<unsigned long> pose_list;
    for (int pose_id = 0; pose_id < keyframes.size(); pose_id++) {
        pose_list.push_back(keyframes.find(pose_id)->second->id_);
        initial_estimate.insert(gtsam::Symbol('x', pose_id), gtsam::Pose3(keyframes.find(pose_id)->second->Pose().matrix()));
        // std::cout << "pose id: " << pose_id << std::endl;
    }

    //此处存疑，不确定是继续用StereoPoint2还是正统Point3
    for (int keypoint_id = 0; keypoint_id < landmarks.size(); keypoint_id++) {
        for (auto observation : landmarks.find(keypoint_id)->second->observations_) {
            //记录右图的像素点横坐标
            if (observation.lock()->is_on_left_image_ == true) {
                float observation_l_x = observation.lock()->position_.pt.x;
                float observation_l_y = observation.lock()->position_.pt.y;
                continue;
            }

            graph.emplace_shared<
                gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> >(gtsam::StereoPoint2(observation_l_x, observation.lock()->position_.pt.x, observation_l_y), model,
                    gtsam::Symbol('x', observation.lock()->frame_.lock()->id_), gtsam::Symbol('l', landmarks.find(keypoint_id)->second->id_), K);

            // std::cout << "x id - l id: " << observation.lock()->frame_.lock()->id_ << " - " << landmarks.find(keypoint_id)->second->id_ << std::endl;

            if (!initial_estimate.exists(gtsam::Symbol('l', landmarks.find(keypoint_id)->second->id_))) {
                gtsam::Pose3 camPose = initial_estimate.at<gtsam::Pose3>(gtsam::Symbol('x', observation.lock()->frame_.lock()->id_));

                // transformFrom() transforms the input Point3 from the camera pose space,
                // camPose, to the global space
                gtsam::Point3 worldPoint = camPose.transformFrom(gtsam::Point3(landmarks.find(keypoint_id)->second->Pos()));
                initial_estimate.insert(gtsam::Symbol('l', landmarks.find(keypoint_id)->second->id_), worldPoint);
            }
        }
    }

    gtsam::Pose3 first_pose = initial_estimate.at<gtsam::Pose3>(gtsam::Symbol('x', pose_list[0]));
    // constrain the first pose such that it cannot change from its original value
    // during optimization
    // NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
    // QR is much slower than Cholesky, but numerically more stable
    graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3> >(gtsam::Symbol('x', pose_list[0]), first_pose);

    std::cout << "Optimizing" << std::endl;
    // create Levenberg-Marquardt optimizer to optimize the factor graph
    gtsam::LevenbergMarquardtParams params;
    params.orderingType = gtsam::Ordering::METIS;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
    gtsam::Values result = optimizer.optimize();

    std::cout << "Final result sample:" << std::endl;
    gtsam::Values pose_values = result.filter<gtsam::Pose3>();
    pose_values.print("Final camera poses:\n");

    gtsam::Values landmarks_values = result.filter<gtsam::Point3>();
    landmarks_values.print("Final landmark poses:\n");

    //可选:去除outlier，现在只是优化pose和feature
    for ( const gtsam::Values::ConstKeyValuePair& key_value : pose_values ) {
        Eigen::Isometry3d pose_ = key_value.value.cast<Eigen::Isometry3d>();

        keyframes.at(key_value.key)->SetPose(pose_);
    }

    for ( const gtsam::Values::ConstKeyValuePair& key_value : landmarks_values ) {
        Eigen::Vector3d landmark_ = key_value.value.cast<Eigen::Vector3d>();

        landmarks.at(key_value.key)->SetPos(landmark_);
    }
}
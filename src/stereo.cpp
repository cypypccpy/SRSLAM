#include <srslam/stereo.h>

bool Stereo::Init() {
    frontend_ = std::shared_ptr<Frontend>(new Frontend);
    backend_ = std::shared_ptr<Backend>(new Backend);
    map_ = std::shared_ptr<map>(new map);

    std::vector<std::shared_ptr<camera>> cameras_;
    
    CameraParam cameraparam;

    cameraparam = readFromYamlFile("/home/lohse/srslam_ws/src/SRSLAM/config/cam0.yaml");

    Eigen::Matrix<double, 3, 3> K;
    K << cameraparam.m_fx, 0.0, cameraparam.m_cx,
        0.0, cameraparam.m_fy, cameraparam.m_cy,
        0.0, 0.0, 1.0;
    Eigen::Matrix<double, 3, 1> t;
    t << 1.10073, 0.0, 0.0;

    Eigen::Isometry3d begin_pose_;
    begin_pose_.pretranslate(t);
    
    std::shared_ptr<camera> new_camera(new camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                        t.norm(), begin_pose_));
    
    cameras_.push_back(new_camera);

    cameraparam = readFromYamlFile("/home/lohse/srslam_ws/src/SRSLAM/config/cam1.yaml");

    K << cameraparam.m_fx, 0.0, cameraparam.m_cx,
        0.0, cameraparam.m_fy, cameraparam.m_cy,
        0.0, 0.0, 1.0;

    t << 1.10073, 0.0, 0.0;
    
    begin_pose_.pretranslate(t);
    begin_pose_ = begin_pose_.matrix();
    
    std::shared_ptr<camera> new_camera1(new camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                        t.norm(), begin_pose_));
    
    cameras_.push_back(new_camera1);

    ROS_INFO("Stereo Inited");

    frontend_->SetCameras(cameras_.at(0), cameras_.at(1));
    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);

    backend_->SetMap(map_);
    backend_->SetCameras(cameras_[0], cameras_[1]);
    return true;
}


CameraParam  Stereo::readFromYamlFile(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    CameraParam cameraparam_;
    cv::FileNode n = fs["mirror_parameters"];
    cameraparam_.m_xi = static_cast<double>(n["xi"]);

    n = fs["intrinsic_parameters"];
    cameraparam_.m_fx = static_cast<double>(n["fx"]);
    cameraparam_.m_fy = static_cast<double>(n["fy"]);
    cameraparam_.m_cx = static_cast<double>(n["cx"]);
    cameraparam_.m_cy = static_cast<double>(n["cy"]);

    return cameraparam_;
}

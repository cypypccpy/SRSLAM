#include "srslam/backend.h"
#include "srslam/datastruct/feature.h"
#include "srslam/frontend.h"
#include "srslam/datastruct/map.h"

Frontend::Frontend() {
    it_ = new image_transport::ImageTransport(nh_);
    right_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/cam1/image_raw", 1);
    left_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/cam0/image_raw", 1);

    sync_ = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *left_sub_, *right_sub_);
    sync_->registerCallback(boost::bind(&Frontend::RegisterCallBack,this, _1, _2));

    num_features_init_ = 50;
    num_features_ = 50;
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();

    return img;
}

void Frontend::RegisterCallBack(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight) {

    current_imgL_ = getImageFromMsg(msgLeft);
    current_imgR_ = getImageFromMsg(msgRight);

    auto new_frame = frame::CreateFrame();
    new_frame->left_img_ = current_imgL_;
    new_frame->right_img_ = current_imgR_;

    current_frame_ = new_frame;

    switch (status_) {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            //Reset();
            break;
    }

    last_frame_ = current_frame_;
}


//------------------------Track-------------------------

bool Frontend::Track() {
    if (last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    int num_track_last = TrackLastFrame();
    int num_coor_features = FindFeaturesInRight();
    
    tracking_inliers_ = EstimateCurrentPose();

    if (tracking_inliers_ > num_features_tracking_) {
        // tracking good
        status_ = FrontendStatus::TRACKING_GOOD;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // tracking bad
        status_ = FrontendStatus::TRACKING_BAD;
    } else {
        // lost
        status_ = FrontendStatus::LOST;
    }

    InsertKeyframe();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    //if (viewer_) viewer_->AddCurrentFrame(current_frame_);
    return true;
}

int Frontend::TrackLastFrame() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_last, kps_current;
    for (auto &kp : last_frame_->features_left_) {
        if (kp->map_point_.lock()) {
            // use project point
            auto mp = kp->map_point_.lock();
            auto px =
                camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(cv::Point2f(px[0], px[1]));
        } else {
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);
            std::shared_ptr<feature> feature_cur(new feature(current_frame_, kp));
            feature_cur->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.push_back(feature_cur);
            num_good_pts++;
        }
    }

    ROS_INFO("Find %d in the last image.", num_good_pts);
    return num_good_pts;
}

int Frontend::EstimateCurrentPose() {
    gtsam::Values initial_estimate_frontend;
    gtsam::NonlinearFactorGraph graph_frontend;

    // 2 poses
    initial_estimate_frontend.insert(gtsam::Symbol('p', 0), gtsam::Pose3(last_frame_->Pose().matrix()));
    initial_estimate_frontend.insert(gtsam::Symbol('p', 1), gtsam::Pose3(current_frame_->Pose().matrix()));

    // create factor noise model with 3 sigmas of value 1
    const auto model = gtsam::noiseModel::Isotropic::Sigma(3, 1);
    // create stereo camera calibration object with .2m between cameras
    const gtsam::Cal3_S2Stereo::shared_ptr K(
        new gtsam::Cal3_S2Stereo(camera_left_->fx_, camera_left_->fy_, camera_left_->cx_,
                                    camera_left_->cy_, 1, camera_left_->baseline_));


    int uti_features = current_frame_->features_left_.size();
    //create and add stereo factors between last pose (key value 1) and all landmarks
    for (size_t i = 0; i < current_frame_->features_left_.size(); i++) {
        if (current_frame_->features_right_[i] == nullptr || current_frame_->features_left_[i]->map_point_.lock() == nullptr) {
            uti_features--;
            continue;
        }

        for (int p = 0; p < 2; p++) {
            graph_frontend.emplace_shared<gtsam::GenericStereoFactor<gtsam::Pose3,gtsam::Point3> >(
                gtsam::StereoPoint2(current_frame_->features_left_[i]->position_.pt.x, 
                                    current_frame_->features_right_[i]->position_.pt.x, current_frame_->features_left_[i]->position_.pt.y),
                                    model, gtsam::Symbol('p', p), gtsam::Symbol('f', i), K);
        }

        gtsam::Pose3 camPose = initial_estimate_frontend.at<gtsam::Pose3>(gtsam::Symbol('p', 0));
        //transformFrom() transforms the input Point3 from the camera pose space, camPose, to the global space
        // gtsam::Point3 worldPoint = camPose.transformFrom(gtsam::Point3(current_frame_->features_left_[i]->map_point_.lock()->pos_));
        // initial_estimate_frontend.insert(gtsam::Symbol('f', i), worldPoint);
    }
    std::cout << "uti_features: " << uti_features << std::endl;
   
    gtsam::Pose3 current_pose = initial_estimate_frontend.at<gtsam::Pose3>(gtsam::Symbol('p', 1));
    // constrain the first pose such that it cannot change from its original value
    // during optimization
    // NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
    // QR is much slower than Cholesky, but numerically more stable
    graph_frontend.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3> >(gtsam::Symbol('p', 1), current_pose);

    std::cout << "Using BA to solve pnp" << std::endl;
    // create Levenberg-Marquardt optimizer to optimize the factor graph
    gtsam::LevenbergMarquardtParams params;
    params.orderingType = gtsam::Ordering::METIS;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph_frontend, initial_estimate_frontend, params);
    gtsam::Values result = optimizer.optimize();

    std::cout << "Final result sample:" << std::endl;
    gtsam::Values pose_values = result.filter<gtsam::Pose3>();
    pose_values.print("Final camera poses:\n");

    //可选:去除outlier，现在只是优化pose
    Eigen::Isometry3d pose_ = pose_values.end()->value.cast<Eigen::Isometry3d>();
    current_frame_->SetPose(pose_);
}

bool Frontend::InsertKeyframe() {
    if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
        // still have enough features, don't insert keyframe
        return false;
    }
    // current frame is a new keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    std::cout << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_ << std::endl;

    SetObservationsForKeyFrame();
    DetectFeatures();  // detect new features

    // track in right image
    FindFeaturesInRight();
    // triangulate map points
    TriangulateNewPoints();
    // update backend because we have a new keyframe
    
    backend_->UpdateMap();

    //if (viewer_) viewer_->UpdateMap();

    return true;
}

void Frontend::SetObservationsForKeyFrame() {
    for (auto &feat : current_frame_->features_left_) {
        auto mp = feat->map_point_.lock();
        if (mp) mp->AddObservation(feat);
    }
}

int Frontend::TriangulateNewPoints() {
    std::vector<Eigen::Isometry3d> poses{camera_left_->pose(), camera_right_->pose()};
    Eigen::Isometry3d current_pose_Twc = current_frame_->Pose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_left_[i]->map_point_.expired() &&
            current_frame_->features_right_[i] != nullptr) {
            // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
            std::vector<Eigen::Vector3d> points{
                camera_left_->pixel2camera(
                    Eigen::Vector2d(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Eigen::Vector2d(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                auto new_map_point = mappoint::CreateNewMappoint();
                pworld = current_pose_Twc * pworld;
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(
                    current_frame_->features_left_[i]);
                new_map_point->AddObservation(
                    current_frame_->features_right_[i]);

                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
                cnt_triangulated_pts++;
            }
        }
    }
    ROS_INFO("new landmarks: %d", cnt_triangulated_pts);
    return cnt_triangulated_pts;
}

//------------------------StereoInit-------------------------
bool Frontend::StereoInit() {
    int num_features_left = DetectFeatures();
    int num_coor_features = FindFeaturesInRight();
    if (num_coor_features < num_features_init_) {
        return false;
    }

    bool build_map_success = BuildInitMap();
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        /*
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        */
        return true;
    }
    return false;
}

int Frontend::DetectFeatures() {
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, -1);
    }

    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(current_frame_->left_img_, corners, 500, 0.01, 10, mask);

    for(auto corner : corners) {
        keypoints.push_back(cv::KeyPoint(corner, 1.f));
    }

    int cnt_detected = 0;
    for (auto &kp : keypoints) {
        current_frame_->features_left_.push_back(
            std::shared_ptr<feature>(new feature(current_frame_, kp)));
        cnt_detected++;
    }

    ROS_INFO("Detect %d new features", cnt_detected);
    return cnt_detected;
}

int Frontend::FindFeaturesInRight() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &kp : current_frame_->features_left_) {
        kps_left.push_back(kp->position_.pt);
        auto mp = kp->map_point_.lock();
        if (mp) {
            // use projected points as initial guess
            auto px =
                camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_right.push_back(cv::Point2f(px[0], px[1]));
        } else {
            // use same pixel in left iamge
            kps_right.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, current_frame_->right_img_, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            std::shared_ptr<feature> feat(new feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feat);
            num_good_pts++;
        } else {
            current_frame_->features_right_.push_back(nullptr);
        }
    }
    ROS_INFO("Find %d in the right image.", num_good_pts);
    return num_good_pts;
}

bool Frontend::BuildInitMap() {
    
    std::vector<Eigen::Isometry3d> poses{camera_left_->pose(), camera_right_->pose()};
    size_t cnt_init_landmarks = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_right_[i] == nullptr) continue;
        // create map point from triangulation
        std::vector<Eigen::Vector3d> points{
            camera_left_->pixel2camera(
                Eigen::Vector2d(current_frame_->features_left_[i]->position_.pt.x,
                     current_frame_->features_left_[i]->position_.pt.y)),
            camera_right_->pixel2camera(
                Eigen::Vector2d(current_frame_->features_right_[i]->position_.pt.x,
                     current_frame_->features_right_[i]->position_.pt.y))};
        Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

        if (Frontend::triangulation(poses, points, pworld) && pworld[2] > 0) {
            auto new_map_point = mappoint::CreateNewMappoint();
            new_map_point->SetPos(pworld);
            new_map_point->AddObservation(current_frame_->features_left_[i]);
            new_map_point->AddObservation(current_frame_->features_right_[i]);
            current_frame_->features_left_[i]->map_point_ = new_map_point;
            current_frame_->features_right_[i]->map_point_ = new_map_point;
            cnt_init_landmarks++;
            map_->InsertMapPoint(new_map_point);
        }
    }

    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    // backend_->UpdateMap();

    ROS_INFO("Initial map created with %zu map points", cnt_init_landmarks);

    return true;
}

bool Frontend::triangulation(const std::vector<Eigen::Isometry3d> &poses,
                   const std::vector<Eigen::Vector3d> points, Eigen::Vector3d &pt_world) {

    A = Eigen::MatrixXd::Zero(4, 4);

    for (size_t i = 0; i < 2; i++) {
        Eigen::Matrix<double, 3, 4> m = poses[i].matrix().block(0, 0, 3, 4);
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }

    svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    // 判断解的质量，不好就放弃
    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        return true;
    }
    return false;
}

//------------------------Reset-------------------------
bool Frontend::Reset() {
    ROS_INFO("Reset is not implemented. ");
    return true;
}


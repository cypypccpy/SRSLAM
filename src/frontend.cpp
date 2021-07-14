#include "srslam/frontend.h"

Frontend::Frontend() {
    it_ = new image_transport::ImageTransport(nh_);
    right_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/mynteye/right/image_raw", 1);
    left_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/mynteye/left/image_raw", 1);

    sync_ = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *right_sub_, *left_sub_);
    sync_->registerCallback(boost::bind(&Frontend::RegisterCallBack,this, _1, _2));

    gftt_ =
        cv::GFTTDetector::create(150, 0.01, 20);
    num_features_init_ = 50;
    num_features_ = 50;
}

void Frontend::RegisterCallBack(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight) {
    current_imgL_ = cv_bridge::toCvShare(msgLeft, "bgr8")->image;
    current_imgR_ = cv_bridge::toCvShare(msgRight, "bgr8")->image;

    switch (status_) {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            //Track();
            break;
        case FrontendStatus::LOST:
            //Reset();
            break;
    }

    last_imgL_ = current_imgL_;
    last_imgR_ = current_imgR_;
}

/*
//------------------------Track-------------------------
bool Frontend::Track() {
    if (last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_.Pose());
    }

    int num_track_last = TrackLastFrame();
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

    if (viewer_) viewer_->AddCurrentFrame(current_frame_);
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
    Mat error;
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
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}
*/

//------------------------StereoInit-------------------------
bool Frontend::StereoInit() {
    int num_features_left = Frontend::DetectFeatures();
    int num_coor_features = Frontend::FindFeaturesInRight();

    if (num_coor_features < num_features_init_) {
        return false;
    }

    bool build_map_success = Frontend::BuildInitMap();

    return true;
}

int Frontend::DetectFeatures() {
    cv::Mat mask(current_imgL_.size(), CV_8UC1, 255);

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_imgL_, keypoints, mask);
    int cnt_detected = 0;
    for (auto &kp : keypoints) {
        current_left_keypoint_position_.push_back(kp);
        cnt_detected++;
    }

    for (auto &feat : current_left_keypoint_position_) {
        cv::rectangle(mask, feat.pt - cv::Point2f(10, 10),
                      feat.pt + cv::Point2f(10, 10), 0, -1);
    }

    ROS_INFO("Detect %d new features", cnt_detected);
    return cnt_detected;
}

int Frontend::FindFeaturesInRight() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &kp : current_left_keypoint_position_) {
        kps_left.push_back(kp.pt);
        // 待定
        // use same pixel in left image
        kps_right.push_back(kp.pt);

    }

    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        current_imgL_, current_imgR_, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            current_right_keypoint_position_.push_back(kp);
            num_good_pts++;
        }
    }
    ROS_INFO("Find %d in the right image.", num_good_pts);
    return num_good_pts;
}


bool Frontend::BuildInitMap() {
    
    std::vector<Eigen::Isometry3d> poses{camera_left_->pose(), camera_right_->pose()};
    size_t cnt_init_landmarks = 0;
    for (size_t i = 0; i < current_left_keypoint_position_.size(); ++i) {
        if (current_left_keypoint_position_[i].size == 0) continue;
        // create map point from triangulation
        std::vector<Eigen::Vector3d> points{
            camera_left_->pixel2camera(
                Eigen::Vector2d(current_left_keypoint_position_[i].pt.x,
                     current_left_keypoint_position_[i].pt.y)),
            camera_right_->pixel2camera(
                Eigen::Vector2d(current_right_keypoint_position_[i].pt.x,
                     current_right_keypoint_position_[i].pt.y))};
        Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

/*
        if (triangulatePoints(poses[0], poses[1], points, pworld) && pworld[2] > 0) {
            auto new_map_point = MapPoint::CreateNewMappoint();
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
    //backend_->UpdateMap();

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points";
*/
    }
    return true;
    }

bool Frontend::triangulation(const std::vector<Eigen::Isometry3d> &poses,
                   const std::vector<Eigen::Vector3d> points, Eigen::Vector3d &pt_world) {
    Eigen::Matrix<double, -1, -1> A(2 * poses.size(), 4);
    Eigen::Matrix<double, -1, 1> b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i) {
        Eigen::Matrix<double, 3, 4> m = poses[i].matrix().block(0, 0, 3, 4);
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // 解质量不好，放弃
        return true;
    }
    return false;
}


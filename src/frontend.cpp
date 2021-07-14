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
            //Track();
            break;
        case FrontendStatus::LOST:
            //Reset();
            break;
    }

    last_frame_ = current_frame_;
}


//------------------------Track-------------------------
/*
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
            std::shared_ptr<feature> feature(new feature(current_frame_, kp));
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }

    ROS_INFO("Find %d in the last image.", num_good_pts);
    return num_good_pts;
}

*/
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
    gftt_->detect(current_frame_->left_img_, keypoints, mask);
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

        if (triangulation(poses, points, pworld) && pworld[2] > 0) {
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
            //backend_->UpdateMap();

    ROS_INFO("Initial map created with %d map points", cnt_init_landmarks);

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


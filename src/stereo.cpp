#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <srslam/stereo.h>

static long factory_id = 0;

void Stereo::RegisterCallBack(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight) {
    cv::Mat imgL = cv_bridge::toCvShare(msgLeft, "bgr8")->image;
    cv::Mat imgR = cv_bridge::toCvShare(msgRight, "bgr8")->image;

    std::shared_ptr<frame> new_frame(new frame);
    new_frame->id_ = factory_id++;
    new_frame->left_img_ = imgL; 
    new_frame->right_img_ = imgR;

    Frontend frontend;
    frontend.AddFrame(new_frame);
}

bool Stereo::Init() {
    std::shared_ptr<Frontend> frontend_ = std::shared_ptr<Frontend>(new Frontend);
    std::vector<std::shared_ptr<camera>> cameras_;
    /*
    std::ifstream fin("/home/hp-3070/srslam_ws/src/srslam/config/calib.txt");
        if (!fin) {
            ROS_INFO("cannot find /home/hp-3070/srslam_ws/src/srslam/config/calib.txt");
            return false;
        }

        for (int i = 0; i < 4; ++i) {
            char camera_name[3];
            for (int k = 0; k < 3; ++k) {
                fin >> camera_name[k];
            }
            double projection_data[12];
            for (int k = 0; k < 12; ++k) {
                fin >> projection_data[k];
            }
            Eigen::Matrix<double, 3, 3> K;
            K << projection_data[0], projection_data[1], projection_data[2],
                projection_data[4], projection_data[5], projection_data[6],
                projection_data[8], projection_data[9], projection_data[10];
            Eigen::Matrix<double, 3, 1> t;
            t << projection_data[3], projection_data[7], projection_data[11];
            
            t = K.inverse() * t;
            K = K * 0.5;
            std::shared_ptr<camera> new_camera(new camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                                t.norm(), Sophus::SE3d(Sophus::SO3d(), t)));
            
            cameras_.push_back(new_camera);
            ROS_INFO("Camera %d extrinsics: ", i);
        }
        fin.close();

    frontend_->SetCameras(cameras_[0], cameras_[1]);
*/
    return true;
}

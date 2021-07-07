#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
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

#include <srslam/frontend.h>

class Stereo
{   
    public:  
        Stereo() {
            it_ = new image_transport::ImageTransport(nh_);
            right_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/mynteye/right/image_raw", 1);
            left_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/mynteye/left/image_raw", 1);

            sync_ = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *right_sub_, *left_sub_);
            sync_->registerCallback(boost::bind(&Stereo::RegisterCallBack,this, _1, _2));
        }

        void RegisterCallBack(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight);
        
        /// 初始化，返回是否成功
        bool Init();

        ~Stereo(){}

    private:
        ros::NodeHandle nh_;  
        image_transport::ImageTransport* it_;
        image_transport::CameraPublisher camera_pub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> sync_pol;
        message_filters::Subscriber<sensor_msgs::Image>* right_sub_ ;
        message_filters::Subscriber<sensor_msgs::Image>* left_sub_;

        message_filters::Synchronizer<sync_pol>* sync_;

        int current_image_index_ = 0;
};  

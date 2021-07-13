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
        Stereo() {};
        
        /// 初始化，返回是否成功
        bool Init();

        ~Stereo(){}

};  

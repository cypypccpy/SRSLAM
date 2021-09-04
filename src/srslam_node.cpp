#include <ros/ros.h>
#include <srslam/stereo.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::AsyncSpinner spinner(1); // single thread
    spinner.start();

    std::shared_ptr<Stereo> stereo_(new Stereo);
    stereo_->Init();

    while (ros::ok()) {
        ros::spinOnce();
    }

    ros::waitForShutdown();
}

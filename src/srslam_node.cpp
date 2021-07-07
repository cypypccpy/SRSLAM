#include <srslam/stereo.h>
#include <srslam/frontend.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::AsyncSpinner spinner(1); // single thread
    spinner.start();

    Stereo stereo_;
    stereo_.Init();

    while (ros::ok()) {
        ros::spinOnce();
    }

    ros::waitForShutdown();
}

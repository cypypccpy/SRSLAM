#include "srslam/datastruct/frame.h"

frame::frame(long id, double time_stamp, const Eigen::Isometry3d &pose, const cv::Mat &left, const cv::Mat &right)
        : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

std::shared_ptr<frame> frame::CreateFrame() {
    static long factory_id = 0;
    std::shared_ptr<frame> new_frame(new frame);
    new_frame->id_ = factory_id++;
    return new_frame;
}

void frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;
    is_keyframe_ = true;
    keyframe_id_ = keyframe_factory_id++;
}


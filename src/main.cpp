#include "rclcpp/rclcpp.hpp"
#include "cv_object_tracker/detector.hpp"
#include "cv_object_tracker/tracker.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor ex;

    rclcpp::NodeOptions det_opts;
    det_opts.arguments({"--ros-args",{"-r"},{"__node:=detector"}});
    rclcpp::NodeOptions track_opts;
    track_opts.arguments({"--ros-args",{"-r"},{"__node:=tracker"}});

    auto detector_node = std::make_shared<cv_tracker::Detector>(det_opts);
    auto tracker_node = std::make_shared<cv_tracker::Tracker>(track_opts);

    ex.add_node(detector_node->get_node_base_interface());
    ex.add_node(tracker_node->get_node_base_interface());

    detector_node->configure();
    tracker_node->configure();

    detector_node->activate();
    tracker_node->activate();

    ex.spin();

    rclcpp::shutdown();
    return 0;
}
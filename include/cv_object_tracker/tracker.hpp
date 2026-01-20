#ifndef CV_OBJECT_TRACKER__TRACKER_HPP_
#define CV_OBJECT_TRACKER__TRACKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_object_tracker/pid.hpp"

namespace cv_tracker{

    class Tracker : public rclcpp_lifecycle::LifecycleNode {

        public:
            explicit Tracker(const rclcpp::NodeOptions &options);
            using CBReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

            CBReturn on_configure(const rclcpp_lifecycle::State &)override;
            CBReturn on_activate(const rclcpp_lifecycle::State &)override;
            CBReturn on_deactivate(const rclcpp_lifecycle::State &)override;
            CBReturn on_cleanup(const rclcpp_lifecycle::State &)override;
        
        private:
            void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
            rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;
            rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
            std::unique_ptr<PID> pid_;
            rclcpp::Time last_time_;
    };
}//namespace

#endif
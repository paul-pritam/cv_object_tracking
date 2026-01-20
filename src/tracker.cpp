#include "cv_object_tracker/tracker.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace cv_tracker {

    Tracker::Tracker(const rclcpp::NodeOptions &opts) : LifecycleNode("tracker", opts) {
        declare_parameter("kp", 0.005);
        declare_parameter("ki", 0.0);
        declare_parameter("kd", 0.0);        
    }

    Tracker::CBReturn Tracker::on_configure(const rclcpp_lifecycle::State &) {
        double kp = get_parameter("kp").as_double();
        double ki = get_parameter("ki").as_double();
        double kd = get_parameter("kd").as_double();

        pid_ = std::make_unique<PID>(kp, ki, kd, -1.0, 1.0);
        pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        return CBReturn::SUCCESS;
    }

    Tracker::CBReturn Tracker::on_activate(const rclcpp_lifecycle::State &) {
        pub_->on_activate();
        pid_->reset();
        last_time_ = get_clock()->now();
        sub_ = create_subscription<vision_msgs::msg::Detection2DArray>("detections", 10, std::bind(&Tracker::detection_callback, this, std::placeholders::_1));
        return CBReturn::SUCCESS;
    }

    Tracker::CBReturn Tracker::on_cleanup(const rclcpp_lifecycle::State &) {
        pub_.reset();
        pid_.reset();
        return CBReturn::SUCCESS;
    }

    Tracker::CBReturn Tracker::on_deactivate(const rclcpp_lifecycle::State &) {
        pub_->on_deactivate();
        sub_.reset();
        geometry_msgs::msg::Twist stop;
        pub_->publish(stop);
        return CBReturn::SUCCESS;
    }

    void Tracker::detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        if (msg->detections.empty() || !pub_->is_activated()){
            return;
        }

        rclcpp::Time now = get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        double target = msg->detections[0].bbox.size_x/2.0;
        double actual = msg->detections[0].bbox.center.position.x;
        double rot = pid_->calculate(target, actual, dt);

        geometry_msgs::msg::Twist t;
        t.angular.z = rot;
        if (std::abs(target - actual) < 20){
            t.linear.x = 0.2;
        }
        pub_->publish(t);
    }
}// namespace
RCLCPP_COMPONENTS_REGISTER_NODE(cv_tracker::Tracker)
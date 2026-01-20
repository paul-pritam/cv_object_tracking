#ifndef CV_OBJECT_TRACKER__DETECTOR_HPP_
#define CV_OBJECT_TRACKER__DETECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "image_transport/image_transport.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"

namespace cv_tracker{

    class Detector : public rclcpp_lifecycle::LifecycleNode {
        public:
            explicit Detector(const rclcpp::NodeOptions &options);
            using CBReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

            CBReturn on_configure(const rclcpp_lifecycle::State &)override;
            CBReturn on_activate(const rclcpp_lifecycle::State &)override;
            CBReturn on_deactivate(const rclcpp_lifecycle::State &)override;
            CBReturn on_cleanup(const rclcpp_lifecycle::State &)override;    
        
        private:
            void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

            rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
            int h_min_, h_max_, s_min_, s_max_, v_min_, v_max_;
    };
}

#endif
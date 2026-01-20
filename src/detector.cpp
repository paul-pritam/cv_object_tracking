#include "cv_object_tracker/detector.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace cv_tracker {

    Detector::Detector(const rclcpp::NodeOptions &opts) :LifecycleNode("detector", opts){
        declare_parameter("h_min", 100);
        declare_parameter("s_min", 150);
        declare_parameter("v_min", 50);

        declare_parameter("h_max", 140);
        declare_parameter("s_max", 255);
        declare_parameter("v_max", 255);
    }

    Detector::CBReturn Detector::on_configure(const rclcpp_lifecycle::State &){
        h_min_ = get_parameter("h_min").as_int();
        s_min_ = get_parameter("s_min").as_int();
        v_min_ = get_parameter("v_min").as_int();

        h_max_ = get_parameter("h_max").as_int();
        s_max_ = get_parameter("s_max").as_int();
        v_max_ = get_parameter("v_max").as_int(); 
        
        pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("detections", 10);
        return CBReturn::SUCCESS;
    }

    Detector::CBReturn Detector::on_activate(const rclcpp_lifecycle::State &) {
        pub_->on_activate();
        sub_ = create_subscription<sensor_msgs::msg::Image>("/camera/image", rclcpp::SensorDataQoS(), std::bind(&Detector::image_callback, this, std::placeholders::_1));
        return CBReturn::SUCCESS;
    }

    Detector::CBReturn Detector::on_deactivate(const rclcpp_lifecycle::State &){
        pub_-> on_deactivate();
        sub_.reset();
        return CBReturn::SUCCESS;
    }

    Detector::CBReturn Detector::on_cleanup(const rclcpp_lifecycle::State &){
        pub_.reset();
        return CBReturn::SUCCESS;
    }

    void Detector::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
        if (!pub_->is_activated()){
            return;
        }
        cv_bridge::CvImagePtr ptr;
        try {
            ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception &e) { 
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return; 
        }
    catch (std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Standard exception: %s", e.what());
        return;
        }

        cv::Mat hsv, mask;
        cv::cvtColor(ptr->image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), mask);
        //mask cleanup
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);  // Removes small noise

        std::vector<std::vector<cv::Point>> cts;
        cv::findContours(mask, cts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!cts.empty()){
            auto max = std::max_element(cts.begin(), cts.end(), [](const auto &a, const auto &b)
            { return cv::contourArea(a)< cv::contourArea(b);});
            cv::Rect r = cv::boundingRect(*max);

            vision_msgs::msg::Detection2DArray out;
            vision_msgs::msg::Detection2D d;

            out.header = msg->header;
            d.bbox.center.position.x = r.x + r.width/2;
            d.bbox.size_x = msg -> width; 
            out.detections.push_back(d);
            pub_->publish(out);       
        }
    }

}//namespace
RCLCPP_COMPONENTS_REGISTER_NODE(cv_tracker::Detector)
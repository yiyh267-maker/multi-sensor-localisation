#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

class YuyvToBgrNode : public rclcpp::Node
{
public:
    YuyvToBgrNode() : Node("yuyv_to_bgr_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&YuyvToBgrNode::imageCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/image_raw_bgr",
            rclcpp::SensorDataQoS());

        RCLCPP_INFO(this->get_logger(), "C++ YUYV -> BGR converter started.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv::Mat yuyv_img;

            // Accept the two supported names for packed YUYV input.
            if (msg->encoding == "yuv422_yuy2" || msg->encoding == "yuyv")
            {
                yuyv_img = cv::Mat(
                    static_cast<int>(msg->height),
                    static_cast<int>(msg->width),
                    CV_8UC2,
                    const_cast<unsigned char*>(msg->data.data()),
                    static_cast<size_t>(msg->step));
            }
            else
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Unsupported input encoding: %s",
                    msg->encoding.c_str());
                return;
            }

            cv::Mat bgr_img;
            cv::cvtColor(yuyv_img, bgr_img, cv::COLOR_YUV2BGR_YUY2);

            auto out_msg = cv_bridge::CvImage(
                msg->header,
                "bgr8",
                bgr_img).toImageMsg();

            pub_->publish(*out_msg);
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Conversion failed: %s",
                e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YuyvToBgrNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

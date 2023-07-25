#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Included String msg defination

static const std::string OPENCV_WINDOW = "Image window";

namespace my_opencv_demo
{
class CppOpenCvDemo : public rclcpp::Node
{
public:
  explicit CppOpenCvDemo(const rclcpp::NodeOptions& options) : rclcpp::Node("topics_demo_node", options)
  {
    publisher_ = create_publisher<sensor_msgs::msg::Image>("/opencv_image", rclcpp::SystemDefaultsQoS());
    // create_publisher method takes in name of the topic and a default Quality of service settings
    subscription_ =
        create_subscription<sensor_msgs::msg::Image>("/rgb/image_raw",
                                                     // name of the topic
                                                     rclcpp::SystemDefaultsQoS(),
                                                     // QoS setting we want to use
                                                     std::bind(&CppOpenCvDemo::Callback, this, std::placeholders::_1)
                                                     // A callback we want to use when ever a new message is received
                                                     // Using std::bind to pass it a member function
                                                     // in this case we can to call a function namaed callback
                                                     // "this" means on the current object
                                                     // placeholder will let it pass through the message parameter

        );
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  // Added mumber variable publisher to our class
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  void Callback(const sensor_msgs::msg::Image::SharedPtr msg)
  // Defination of the call back when a new message is received
  {
    sensor_msgs::msg::Image greeting_msg;
    sensor_msgs::msg::Image::SharedPtr msg_;
    // A new message object "greeting_msg"
    greeting_msg.data = msg->data;
    greeting_msg.encoding = msg->encoding;
    greeting_msg.header = msg->header;
    greeting_msg.width = msg->width;
    greeting_msg.step = msg->step;
    greeting_msg.height = msg->height;
    greeting_msg.is_bigendian = msg->is_bigendian;

    cv_bridge::CvImagePtr cv_ptr;

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if ((cv_ptr->image.rows > 60) && (cv_ptr->image.cols > 60))
    {
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
    }

    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_ptr->image).toImageMsg();

    publisher_->publish(*msg_.get());

    // populating the data field using the datafield of incomming message
    // publisher_->publish(greeting_msg);
    // Publishing a new message using publisher
  }  // Logic to take the name message add greeting and publish it
};

}  // namespace my_opencv_demo

RCLCPP_COMPONENTS_REGISTER_NODE(my_opencv_demo::CppOpenCvDemo)
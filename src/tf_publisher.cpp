/**
 * @file tf_publisher.cpp
 * @author Rashmi Kapu (rashmik@umd.edu)
 * @brief  A simple publisher that creates a topic called 'topic'
 and publishes to the topic a customised String message every 500ms and also broadcasts
 a TF frame transformation (child node).
 * @version 0.1
 * @date 2023-11-21
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
// #include "turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;

/**
 * @brief This example creates a subclass of Node and uses std::bind() to
 * register a member function as a callback from the timer (every 500ms).
 */

class TFPublisher : public rclcpp::Node {
 public:
  TFPublisher(char* frame_coord[]) : Node("tf_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);


    timer_ = this->create_wall_timer(
        500ms, std::bind(&TFPublisher::timer_callback, this));
    
    this->handle_turtle_pose(frame_coord);
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data =
        "Hello, world! This is Rashmi Kapu!! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }


/**
 * @brief This function creates a child frame and broadcasts the transformation.
 * 
 * @param msg 
 */
   void handle_turtle_pose(char** msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = msg[1];
    

    // We get x y z translation
    // coordinates from the message 
    t.transform.translation.x = std::atof(msg[2]);
    t.transform.translation.y = std::atof(msg[3]);
    t.transform.translation.z = std::atof(msg[4]);

    // We set rotation in x and y and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(atof(msg[5]), atof(msg[6]), atof(msg[7]));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char* argv[]) {
  auto logger = rclcpp::get_logger("logger");

  // Obtain parameters from command line arguments
  if (argc != 8) {
    RCLCPP_INFO(
      logger, "Invalid number of parameters\nusage: "
      "$ ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster "
      "child_frame_name x y z roll pitch yaw");
    return 1;
  }

  // As the parent frame of the transform is `world`, it is
  // necessary to check that the frame name passed is different
  if (strcmp(argv[1], "world") == 0) {
    RCLCPP_INFO(logger, "Your static turtle name cannot be 'world'");
    return 1;
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFPublisher>(argv));
  rclcpp::shutdown();
  return 0;
}

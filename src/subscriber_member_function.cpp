/**
 * @file subscriber_member_function.cpp
 * @author Rashmi Kapu (rashmik@umd.edu)
 * @brief A simple subscriber that subscribes to the topic named 'topic' 
 to read the messages of type 'std_msgs::msg::String' and print on the console.
 * @version 0.1
 * @date 2023-11-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

/**
 * @brief MinimalSubscriber class node that extends the rclcpp class
 It reads messages from the topic 'topic' and prints on the screen.
 * 
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

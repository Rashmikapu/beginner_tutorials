/**
 * @file param_subscriber.cpp
 * @author Rashmi Kapu (rashmik@umd.edu)
 * @brief This is the Parameter subscriber node that subscribes to '/chatter'
 topic, to which parameter publisher publishes.
 * @version 0.1
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

/**
 * @brief ParamSubscriber class node that extends the rclcpp class
 It reads messages from the topic '/chatter' and prints on the screen.
 *
 */
class ParamSubscriber : public rclcpp::Node {
 public:
  ParamSubscriber() : Node("param_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&ParamSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief This is the subscriber callback function.
   *
   * @param msg
   */
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard from param publisher: '%s'",
                msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParamSubscriber>());
  rclcpp::shutdown();
  return 0;
}

/**
 * @file param_publisher.cpp
 * @author Rashmi Kapu (rashmik@umd.edu)
 * @brief This is a parameterized publisher node that takes timer time_period (frequency)
 as the parameter and calls the callback function accordingly.
 * @version 0.1
 * @date 2023-11-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/clock.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @brief This example creates a subclass of Node and uses std::bind() to
 * register a member function as a callback from the timer (based on freq param).
 */

class ParamPublisher : public rclcpp::Node {
 public:
  ParamPublisher() : Node("param_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    this->declare_parameter("freq", 1.0);

    // auto freq = std::stof(this->get_parameter("freq").as_string());
    auto freq = this->get_parameter("freq").as_double();

    // Logger levels
    if (freq) {
      // Fatal
      if (freq <= 0) {
        RCLCPP_FATAL_STREAM(
            this->get_logger(),
            "Frequency value cannot be less than or equal to zero!");
      } else if (freq > 200) {
      // Warning
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Publishing frequency too high!");
      } else {
      // Debug- no more than once per second, skipping the very
      // first time it is hit:
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Frequency: " << freq);
      }
    }
    auto timer_period =
        std::chrono::milliseconds(static_cast<int>(1000.0 / freq));
    timer_ = this->create_wall_timer(
        timer_period, std::bind(&ParamPublisher::timer_callback, this));
  }

 private:
 /**
  * @brief  This is the timer_callback function called by the param publisher. It publishes the
  message to the topic '/chatter'.
  * 
  */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! This is a parameter publisher!! " +
                   std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    // std::vector<rclcpp::Parameter>
    // all_new_parameters{rclcpp::Parameter("freq", "1.0")};
    // this->set_parameters(all_new_parameters);
    // if(this->get_parameter("freq") != 1.0){
    //     RCLCPP_ERROR_STREAM_THROTTLE(
    //     this->get_logger(),*get_node_base_interface(), 1000,"Parameter has
    //     not been changed back to default"<< 4);
    // }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParamPublisher>());
  rclcpp::shutdown();
  return 0;
}

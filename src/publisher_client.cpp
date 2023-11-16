/**
 * @file publisher_client.cpp
 * @author Rashmi Kapu (rashmik@umd.edu)
 * @brief This is the Client for the service. It also contains a publisher
 which publishes the modified string to the topic.
 * @version 0.1
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief This class is a node that is a client to ChangeString service.
 Each time it calls a binded timer_callback and requests for a modified input,
 the response from the server is collected and published to the topic using this
 node.
 *
 */
class ServicePublisher : public rclcpp::Node {
 public:
  ServicePublisher() : Node("service_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&ServicePublisher::timer_callback, this));
    // auto message = std_msgs::msg::String();
    client_ =
        create_client<beginner_tutorials::srv::ChangeString>("ChangeString");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        // Used one of the RCLCPP LOG Level
        RCLCPP_ERROR(this->get_logger(),
                     "Interruped while waiting for the server.");

        return;
      }
      RCLCPP_INFO(this->get_logger(), "Server not available, waiting again...");
    }
  }

  /**
   * @brief This callback requests the server
   *
   */
 private:
  void timer_callback() {
    auto request =
        std::make_shared<beginner_tutorials::srv::ChangeString::Request>();
    request->input =
        "Hello, world! This is Rashmi Kapu!! " + std::to_string(count_++);

    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'",
    // ((request->input).c_str())); publisher_->publish(message);

    // Create a pointer res to a callback function.
    auto res = std::bind(&ServicePublisher::service_publish, this, _1);
    // Everytime a request is sent to the service, the callback function
    // pointed by the second argument is executed. Here it is 'res'.
    client_->async_send_request(request, res);
    // auto result = client->async_send_request(request);
    // auto node = shared_from_this();
  }

  /**
   * @brief This callback publishes the response to the topic.
   *
   * @param future
   */
  void service_publish(
      rclcpp::Client<beginner_tutorials::srv::ChangeString>::SharedFuture
          future) {
    auto response = std_msgs::msg::String();

    response.data = future.get()->output.c_str();
    RCLCPP_INFO(this->get_logger(), "Publishing : '%s'", response.data.c_str());
    publisher_->publish(response);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Client<beginner_tutorials::srv::ChangeString>::SharedPtr client_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServicePublisher>());
  rclcpp::shutdown();
  return 0;
}

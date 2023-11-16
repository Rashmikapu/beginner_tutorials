/**
 * @file change_string_service.cpp
 * @author Rashmi Kapu (rashmik@umd.edu)
 * @brief This is a service node that is invoked when a client requests
 for a change of string. This adds a string to the request and sends the
 response.
 * @version 0.1
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <string>

#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Function that adds a string to the incoming request and sends
 the response.
 *
 * @param request
 * @param response
 */
void change(
    const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
        request,
    std::shared_ptr<beginner_tutorials::srv::ChangeString::Response> response) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request");
  response->output = static_cast<std::string>(request->input.c_str()) +
                     ": Created by custom service";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: %s",
              response->output.c_str());
}
/**
 * @brief Calls the change function everytime client requests for the
 service.
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("ChangeString_server");

  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service =
      node->create_service<beginner_tutorials::srv::ChangeString>(
          "ChangeString", &change);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to change string");

  rclcpp::spin(node);
  rclcpp::shutdown();
}

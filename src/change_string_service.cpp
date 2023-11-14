#include "rclcpp/rclcpp.hpp"
#include "beginner_tutorials/srv/change_string.hpp"

#include <memory>
#include <string>

void change(const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request> request,
          std::shared_ptr<beginner_tutorials::srv::ChangeString::Response>      response)
{

  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request");
  response->output = static_cast<std::string>(request->input.c_str()) + ": Created by custom service" ;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: %s", response->output.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ChangeString_server");

  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service =
    node->create_service<beginner_tutorials::srv::ChangeString>("ChangeString", &change);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to change string");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
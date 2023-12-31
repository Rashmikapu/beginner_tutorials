/**
 * @file testcases.cpp
 * @author Rashmi Kapu  (rashmik@umd.edu)
 * @brief This file is written to test the package beginner_tutorials
 (Specifically the talker node)
 * @version 0.1
 * @date 2023-11-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <gtest/gtest.h>
#include <stdlib.h>

#include <std_msgs/msg/string.hpp>

/**
 * @brief This node extends Test to start and stop the talker node
 and verify the output from the topic.
 *
 */
class TalkerIntegrationTest : public testing::Test {
 public:
  TalkerIntegrationTest()
      : node_(std::make_shared<rclcpp::Node>("talker_integration_test")) {}
  /**
   * @brief Set the Up object
   *
   */
  void SetUp() override {
    // Setup things that should occur before every test instance should go here

    /*
     * Start the Talker node for testing
     */
    bool ret_val =
        StartTalkerNode("beginner_tutorials", "minimal_publisher", "talker");
    ASSERT_TRUE(ret_val);

    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }
  /**
   * @brief To stop the node after running the test
   *
   */
  void TearDown() override {
    // Tear things that should occur after every test instance should go here

    // Stop the running Talker node
    bool ret_val = StopTalkerNode();
    ASSERT_TRUE(ret_val);

    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH TEARDOWN");
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  bool has_data_;
  std::stringstream cmd_ss, cmdInfo_ss, killCmd_ss;

  /**
   * @brief To start the talker node
   *
   * @param pkg_name
   * @param node_name
   * @param exec_name
   * @return true
   * @return false
   */
  bool StartTalkerNode(const char* pkg_name, const char* node_name,
                       const char* exec_name) {
    // Implement the logic to start the Talker node for testing

    // build command strings
    cmd_ss << "ros2 run " << pkg_name << " " << exec_name
           << " > /dev/null 2> /dev/null &";
    cmdInfo_ss << "ros2 node info "
               << "/" << node_name << " > /dev/null 2> /dev/null";
    char execName[16];
    snprintf(execName, 16, "%s",
             exec_name);  // pkill uses exec name <= 15 char only
    killCmd_ss << "pkill --signal SIGINT " << execName
               << " > /dev/null 2> /dev/null";

    // First kill the ros2 node, in case it's still running.
    StopTalkerNode();

    // Start a ros2 node and wait for it to get ready:
    int retVal = system(cmd_ss.str().c_str());
    if (retVal != 0) return false;

    retVal = -1;
    while (retVal != 0) {
      retVal = system(cmdInfo_ss.str().c_str());
      sleep(1);
    }
    return true;  // Return true if the node started successfully, false
                  // otherwise
  }

  /**
   * @brief Kill the node object
   *
   * @return true
   * @return false
   */
  bool StopTalkerNode() {
    // Implement the logic to stop the Talker node

    if (killCmd_ss.str().empty()) return true;

    int retVal = system(killCmd_ss.str().c_str());
    return retVal == 0;
  }
};

TEST_F(TalkerIntegrationTest, TopicPublishingTest) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  /*
   * 2.) subscribe to the topic
   */
  using std_msgs::msg::String;
  using SUBSCRIBER = rclcpp::Subscription<String>::SharedPtr;
  bool hasData = false;
  SUBSCRIBER subscription = node_->create_subscription<String>(
      "topic", 10,
      // Lambda expression begins
      [&](const std_msgs::msg::String& msg) {
        RCLCPP_INFO(node_->get_logger(), "I heard: '%s'", msg.data.c_str());
        hasData = true;
      }  // end of lambda expression
  );

  /*
   * 3.) check to see if we get data winhin 3 sec
   */
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while (elapsed_time < 3s) {
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  EXPECT_TRUE(hasData);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}

/**
 * @file dorahand_ros2.cpp
 * @author Dorabot Inc.
 * @brief ros2 demo for control DoraHand
 * @version 1.1.1
 * @date 2021-02-08
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <cstdio>
#include <unistd.h>
#include <chrono>
#include <thread>
#include "dorahand_ros2_node.hh"

using namespace dr;

int main(int argc, char ** argv)
{
  printf("Hello world dorahand_ros2 package\n");
  std::vector<std::string> arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);

  printf("New DoraHand\n");
  auto node = std::make_shared<dorahand_ros2_node::DorahandRos2Node>();

  printf("spin\n");
  rclcpp::spin(node);

  return 0;
}
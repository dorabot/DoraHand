/**
 * @file dorahand_ros.hh
 * @author Dorabot Inc.
 * @brief ros2 demo for control DoraHand
 * @version 1.1.1
 * @date 2021-02-08
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __DORAHAND_ROS_NODE_HH__
#define __DORAHAND_ROS_NODE_HH__

#include <string>
#include <functional>

#include <urdf/model.h>

#include <dorahand.hh>
#include <dorahand_ros/ControlJoint.h>
#include <dorahand_ros/GetHandState.h>
#include <dorahand_ros/GraspControl.h>
#include <dorahand_ros/SetHandState.h>

enum error_code
{
  SUCCESS = 1,
  TIMEOUT = -1,
  FAILURE = 0,
};

void create_services();
bool set_init(const nlohmann::json &config);
void command_map_init();

bool control_joint(dorahand_ros::ControlJoint::Request &request,
dorahand_ros::ControlJoint::Response &response);

bool get_hand_state(dorahand_ros::GetHandState::Request &request,
dorahand_ros::GetHandState::Response &response);

bool grasp_control(dorahand_ros::GraspControl::Request &request,
dorahand_ros::GraspControl::Response &response);

bool set_hand_state(dorahand_ros::SetHandState::Request &request,
dorahand_ros::SetHandState::Response &response);

IO_SERCAN_HAND ee_obj_; 
   
std::atomic<bool> query_thread_run_;

std::vector<std::string> command_name_ =
{
  "command_clear_screen",
  "command_stop_hand",
  "command_reset_hand",
  "command_release_hand",
  "command_read_main_version",
  "command_read_finger_version",
  "command_read_product_type",
  "command_read_report_rate",
  "command_paralle_change_posture",
  "command_paralle_grasp_distance",
  "command_paralle_grasp_current",
  "command_control_force",
  "command_control_current",
  "command_control_position",
  "command_control_individual",
  "command_set_max_current",
  "command_set_speed",
  "command_set_report_rate",
  "command_control_5F_diff_joint"
};
std::map<int,std::string> command_map_;

DexterousHandMessage *dh_message_; 

#endif // __DORAHAND_ROS_NODE_HH__
/**
 * @file dorahand_ros2_node.hh
 * @author Dorabot Inc.
 * @brief ros2 demo for control DoraHand
 * @version 1.1.1
 * @date 2021-02-08
 *
 * @copyright Copyright (c) 2021
 *
 */
 
#ifndef __DORAHAND_ROS2_NODE_HH__
#define __DORAHAND_ROS2_NODE_HH__

#include <string>
#include <functional>

#include <urdf/model.h>
#include <rclcpp/rclcpp.hpp>

#include <dorahand.hh>
#include <dorahand_interfaces_ros2/srv/control_joint.hpp>
#include <dorahand_interfaces_ros2/srv/get_hand_state.hpp>
#include <dorahand_interfaces_ros2/srv/grasp_control.hpp>
#include <dorahand_interfaces_ros2/srv/set_hand_state.hpp>

namespace dr
{
  namespace dorahand_ros2_node
  {
    enum error_code
    {
      SUCCESS = 1,
      TIMEOUT = -1,
      FAILURE = 0,
    };

    class DorahandRos2Node : public rclcpp::Node
    {
    public:
      explicit DorahandRos2Node(const std::string& service_name = "dorahand_ros2_service");
      ~DorahandRos2Node()
      {
        delete dh_message_;
      }

      void create_services();
      bool set_init(const nlohmann::json &config);
      void command_map_init();

    public:    
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

    private:
      void control_joint(const std::shared_ptr<dorahand_interfaces_ros2::srv::ControlJoint::Request> request,
      std::shared_ptr<dorahand_interfaces_ros2::srv::ControlJoint::Response> response);

      void get_hand_state(const std::shared_ptr<dorahand_interfaces_ros2::srv::GetHandState::Request> request,
      std::shared_ptr<dorahand_interfaces_ros2::srv::GetHandState::Response> response);

      void grasp_control(const std::shared_ptr<dorahand_interfaces_ros2::srv::GraspControl::Request> request,
      std::shared_ptr<dorahand_interfaces_ros2::srv::GraspControl::Response> response);

      void set_hand_state(const std::shared_ptr<dorahand_interfaces_ros2::srv::SetHandState::Request> request,
      std::shared_ptr<dorahand_interfaces_ros2::srv::SetHandState::Response> response);

    private:
      rclcpp::Service<dorahand_interfaces_ros2::srv::ControlJoint>::SharedPtr control_joint_srv_;
      rclcpp::Service<dorahand_interfaces_ros2::srv::GetHandState>::SharedPtr get_hand_state_srv_;
      rclcpp::Service<dorahand_interfaces_ros2::srv::GraspControl>::SharedPtr grasp_control_srv_;
      rclcpp::Service<dorahand_interfaces_ros2::srv::SetHandState>::SharedPtr set_hand_state_srv_;
      rclcpp::callback_group::CallbackGroup::SharedPtr cb_dr1_;

      DexterousHandMessage *dh_message_; 
    }; // class DorahandRos2Node
  } // namespace dorahand_ros2_node
} // namespace dr

#endif // __DORAHAND_ROS2_NODE_HH__
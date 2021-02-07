/**
 * @file dorahand_ros2_node.cc
 * @author Dorabot Inc.
 * @brief ros2 demo for control DoraHand
 * @version 1.1.1
 * @date 2021-02-08
 *
 * @copyright Copyright (c) 2021
 *
 */
 
#include "dorahand_ros2_node.hh"
#include <ament_index_cpp/get_package_share_directory.hpp>
namespace dr
{
  namespace dorahand_ros2_node
  {
    DorahandRos2Node::DorahandRos2Node(const std::string& service_name):Node(service_name)
    {
      nlohmann::json config_sercan;
      // std::ifstream input_sercan("/etc/dorabot/dorahand/config.json");
      // input_sercan >> config_sercan;

      dh_message_ = new DexterousHandMessage();
      create_services();
      set_init(config_sercan);

      query_thread_run_ = false;
    }

    bool DorahandRos2Node::set_init(const nlohmann::json &config)
    {
      urdf::Model urdf;
      command_map_init();

      ee_obj_.on_init();
      ee_obj_.set_callback();
      ee_obj_.on_monitor();

      EndEffectorCommand cmd;
      DexterousHandMessage *message = new DexterousHandMessage ();
      DexterousHandFeedbackMessage *fb_message = new DexterousHandFeedbackMessage ();

      // Constructs the new thread and runs it. Does not block execution.
      query_thread_run_ = true;
      std::thread query_hand_thread{[&]() {
        // query data
        bool ret = false;
        while (query_thread_run_.load ())
        {
          std::this_thread::sleep_for (std::chrono::milliseconds (5));
          ret = ee_obj_.read_hand_status (message);
          if (ret)
          {
            message->Clear (); 
          }
        }
      }};
      query_hand_thread.detach();

      return true;
    }

    void DorahandRos2Node::command_map_init()
    { 
      command_map_.clear();
      for(uint command_index = 0; command_index < command_name_.size(); command_index++)
      {
        command_map_.insert(std::pair<int,std::string>(command_index, command_name_[command_index]));
      }
    }

    void DorahandRos2Node::create_services()
    {
      cb_dr1_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);

      get_hand_state_srv_ = this->create_service<dorahand_interfaces_ros2::srv::GetHandState>(
          "dorahand_service/get_hand_state",
          std::bind(
              &DorahandRos2Node::get_hand_state,
              this,
              std::placeholders::_1,
              std::placeholders::_2),
          rmw_qos_profile_services_default,
          cb_dr1_);
      set_hand_state_srv_ = this->create_service<dorahand_interfaces_ros2::srv::SetHandState>(
          "dorahand_service/set_hand_state",
          std::bind(
              &DorahandRos2Node::set_hand_state,
              this,
              std::placeholders::_1,
              std::placeholders::_2),
          rmw_qos_profile_services_default,
          cb_dr1_);
      grasp_control_srv_ = this->create_service<dorahand_interfaces_ros2::srv::GraspControl>(
          "dorahand_service/grasp_control",
          std::bind(
              &DorahandRos2Node::grasp_control,
              this,
              std::placeholders::_1,
              std::placeholders::_2),
          rmw_qos_profile_services_default,
          cb_dr1_);
      control_joint_srv_ = this->create_service<dorahand_interfaces_ros2::srv::ControlJoint>(
          "dorahand_service/control_joint",
          std::bind(
              &DorahandRos2Node::control_joint,
              this,
              std::placeholders::_1,
              std::placeholders::_2),
          rmw_qos_profile_services_default,
          cb_dr1_);
    }

    void DorahandRos2Node::get_hand_state(const std::shared_ptr<dorahand_interfaces_ros2::srv::GetHandState::Request> request,
                                          std::shared_ptr<dorahand_interfaces_ros2::srv::GetHandState::Response> response)
    {
      std::cout << "get_hand_state: " << request->request << std::endl;
    }

    void DorahandRos2Node::set_hand_state(const std::shared_ptr<dorahand_interfaces_ros2::srv::SetHandState::Request> request,
                                          std::shared_ptr<dorahand_interfaces_ros2::srv::SetHandState::Response> response)
    {
      if(request->mode == 0)
      {
        ee_obj_.hand_stop();
      }
      else if (request->mode == 1)
      {
        ee_obj_.hand_reset();
      }
      else if (request->mode == 2)
      {
        ee_obj_.hand_release();
      }
      response->error_code = error_code::SUCCESS;
    }

    void DorahandRos2Node::grasp_control(const std::shared_ptr<dorahand_interfaces_ros2::srv::GraspControl::Request> request,
                                          std::shared_ptr<dorahand_interfaces_ros2::srv::GraspControl::Response> response)
    {
      if (request->mode == 0)
      {
        std::cout << "grasp mode: " << request->mode << " distance: " << request->command[0] << std::endl;
        if(request->mode < 0 || request->mode > 2)
        {
          response->error_code = error_code::FAILURE;
        }
        else
        {
          ee_obj_.hand_paralle_grasp_distance(request->gesture, request->command[0]);
          response->error_code = error_code::SUCCESS;
        }
      }
      else if (request->mode == 1)
      {
        std::cout << "grasp mode: " << request->mode << " current0: " << request->command[0] << " current1: " << request->command[1] << std::endl;
        if(request->mode < 0 || request->mode > 2)
        {
          response->error_code = error_code::FAILURE;
        }
        else
        {
          ee_obj_.hand_paralle_grasp_current(request->gesture, request->command[0], request->command[1]);
          response->error_code = error_code::SUCCESS;
        }
      }
    }

    void DorahandRos2Node::control_joint(const std::shared_ptr<dorahand_interfaces_ros2::srv::ControlJoint::Request> request,
                                          std::shared_ptr<dorahand_interfaces_ros2::srv::ControlJoint::Response> response)
    {
      std::cout << "control joint mode: " << request->command[0] << std::endl;
      if(request->command[0] < 0 || request->command[0] > 3)
      {
          response->error_code = error_code::FAILURE;
      }
      else
      {
          ee_obj_.hand_joint_control(request->command);
          response->error_code = error_code::SUCCESS;
      }
    }
  } // namespace dorahand_ros2_node
} // namespace dr

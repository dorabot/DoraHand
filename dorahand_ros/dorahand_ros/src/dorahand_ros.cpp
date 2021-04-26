/**
 * @file dorahand_ros.cpp
 * @author Dorabot Inc.
 * @brief ros demo for control DoraHand
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
#include <fstream>
#include "dorahand_ros.hh"

using namespace dr;

bool set_init(const nlohmann::json &config)
{
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
        hand_data_pub(message);
        message->Clear(); 
      }
    }
  }};
  query_hand_thread.detach();

  return true;
}

void hand_data_pub(DexterousHandMessage *message)
{ 
  for (int i = 0; i < message->fingers_size (); i++)
  {
    if (message->fingers(i).finger_id() == PALM_AREA_ID)
    {
      update_finger_data(PALM_AREA_ID, &message->fingers(i));
    }
    else if (message->fingers(i).finger_id() == LF_AREA_ID)
    {
      update_finger_data(LF_AREA_ID, &message->fingers(i));
    }
    else if (message->fingers(i).finger_id() == RF_AREA_ID)
    {
      update_finger_data(RF_AREA_ID, &message->fingers(i));
    }
    else if (message->fingers(i).finger_id() == MF_AREA_ID)
    {
      update_finger_data(MF_AREA_ID, &message->fingers(i));
    }
  }
}

void update_finger_data(int F_ID, const DexterousHandFingerMessage *message)
{
  if (message->joint_id() == PJ_ID)
  {
    update_joint_data(F_ID, PJ_ID, message);
  }
  else if (message->joint_id() == MJ_ID)
  {
    update_joint_data(F_ID, MJ_ID, message);
  }
}

void update_joint_data(int F_ID, int J_ID, const DexterousHandFingerMessage *message)
{
  float draw_data;
  hand_msg_.data.push_back(F_ID);
  hand_msg_.data.push_back(J_ID);
  draw_data = message->position();
  hand_msg_.data.push_back(draw_data);
  draw_data = message->velocity();
  hand_msg_.data.push_back(draw_data);
  draw_data = message->current();
  hand_msg_.data.push_back(draw_data);
  hand_msg_.data.push_back(0);
  for (int i=0;i<message->force_size();i++)
  {
    draw_data = message->force(i);
    switch(i)
    {
      case 0:
        hand_msg_.data.push_back(draw_data);
        break;
      case 1:
        hand_msg_.data.push_back(draw_data);
        break;
      case 2:
        hand_msg_.data.push_back(draw_data);
        break;
      case 3:
        hand_msg_.data.push_back(draw_data);
        break;
      default:
        break;
    }
  }
}

void command_map_init()
{ 
  command_map_.clear();
  for(uint command_index = 0; command_index < command_name_.size(); command_index++)
  {
    command_map_.insert(std::pair<int,std::string>(command_index, command_name_[command_index]));
  }
}

bool get_hand_state(dorahand_ros::GetHandState::Request &request,
                    dorahand_ros::GetHandState::Response &response)
{
  std::cout << "get_hand_state: " << request.request << std::endl;
  return true;
}

bool set_hand_state(dorahand_ros::SetHandState::Request &request,
                    dorahand_ros::SetHandState::Response &response)
{
  if(request.mode == 0)
  {
    ee_obj_.hand_stop();
  }
  else if (request.mode == 1)
  {
    ee_obj_.hand_reset();
  }
  else if (request.mode == 2)
  {
    ee_obj_.hand_release();
  }
  response.error_code = error_code::SUCCESS;
  return true;
}

bool grasp_control(dorahand_ros::GraspControl::Request &request,
                  dorahand_ros::GraspControl::Response &response)
{
  if (request.mode == 0)
  {
    std::cout << "grasp mode: " << request.mode << " distance: " << request.command[0] << std::endl;
    if(request.mode < 0 || request.mode > 2)
    {
      response.error_code = error_code::FAILURE;
    }
    else
    {
      ee_obj_.hand_paralle_grasp_distance(request.gesture, request.command[0]);
      response.error_code = error_code::SUCCESS;
    }
  }
  else if (request.mode == 1)
  {
    std::cout << "grasp mode: " << request.mode << " current0: " << request.command[0] << " current1: " << request.command[1] << std::endl;
    if(request.mode < 0 || request.mode > 2)
    {
      response.error_code = error_code::FAILURE;
    }
    else
    {
      ee_obj_.hand_paralle_grasp_current(request.gesture, request.command[0], request.command[1]);
      response.error_code = error_code::SUCCESS;
    }
  }
  return true;
}

bool control_joint(dorahand_ros::ControlJoint::Request &request,
                   dorahand_ros::ControlJoint::Response &response)
{
  std::cout << "control joint mode: " << request.command[0] << std::endl;
  if(request.command[0] < 0 || request.command[0] > 3)
  {
      response.error_code = error_code::FAILURE;
  }
  else
  {
      ee_obj_.hand_joint_control(request.command);
      response.error_code = error_code::SUCCESS;
  }
  return true;
}

int main(int argc, char ** argv)
{
  printf("Hello world dorahand_ros package\n");
  printf("New DoraHand\n");

  ros::init(argc, argv, "dorahand_ros");
  ros::NodeHandle nh;

  ros::Publisher hand_pub = nh.advertise<std_msgs::Float32MultiArray>("/hand_data", 10);
  ros::ServiceServer get_hand_state_srv = nh.advertiseService("dorahand_service/get_hand_state", get_hand_state);
  ros::ServiceServer set_hand_state_srv = nh.advertiseService("dorahand_service/set_hand_state", set_hand_state);
  ros::ServiceServer grasp_control_srv = nh.advertiseService("dorahand_service/grasp_control", grasp_control);
  ros::ServiceServer control_joint_srv = nh.advertiseService("dorahand_service/control_joint", control_joint);

  nlohmann::json config_sercan;
  set_init(config_sercan);

  DexterousHandMessage *message = new DexterousHandMessage();

  query_thread_run_ = false;
  bool ret;

  while(ros::ok())
  {
    ret = false;
    std::this_thread::sleep_for (std::chrono::milliseconds (5));
    ret = ee_obj_.read_hand_status (message);
    if (ret)
    {
      hand_data_pub(message);
      hand_pub.publish(hand_msg_);
      hand_msg_.data.clear();
      message->Clear(); 
    }

    ros::spinOnce();
  }

  return 0;
}
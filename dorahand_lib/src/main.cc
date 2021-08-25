/**
 * @file main.cc
 * @author Dorabot Inc.
 * @brief library for control DoraHand
 * @version 1.1.16
 * @date 2021-8-24
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "rt_key_parameter.pb.h"
#include <dorahand.hh>
#include <io_dr_sercan.hh>
#include <io_sercan_command.hh>
#include <nlohmann/json.hpp>
#include <functional>
#include <iostream>
#include <fstream>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <arpa/inet.h>

// the time duration of every joint command should longer than 50ms

using namespace dr;
#define CONFIG_PATH "/usr/etc/dorabot/dorahand/config.json"

std::vector<std::string> command_name =
{
  "command_clear_screen",
  "command_stop_hand",
  "command_reset_hand",
  "command_release_hand",
  "command_read_main_version",
  "command_read_finger_fw_version",
  "command_read_finger_config_version",
  "command_read_product_type",
  "command_read_report_rate",
  "command_paralle_change_posture",
  "command_paralle_grasp_distance",
  "command_paralle_grasp_current",
  "command_control_position",
  "command_control_current",
  "command_control_force",
  "command_control_individual_3f",
  "command_control_individual_5f",
  "command_set_max_current",
  "command_set_speed",
  "command_set_report_rate",
  "command_control_5F_diff_joint",
  "command_test_case_1",
  "command_rtkp_get",
  "command_rtkp_record",
  "command_rtkp_clear",
  "command_config_local_ip_address",
  "command_config_local_ip_netmask",
  "command_config_local_ip_gateway"
};
std::map<int,std::string> command_map;
void command_map_init()
{ 
  command_map.clear();
  for(uint command_index=0;command_index<command_name.size();command_index++)
  {
    command_map.insert(std::pair<int,std::string>(command_index,command_name[command_index]));
  }
} 

std::mutex monitor_lock_;

const char* dexterous_program_name;
void dexterous_args_update( int argc, char **argv);

int main (int argc, char **argv)
{
  int rst = -1;
  dexterous_args_update(argc, argv);

  command_map_init();
  
  IO_SERCAN_HAND ee_obj;

  ee_obj.on_init();
  ee_obj.set_callback();
  ee_obj.on_monitor();

  dr::EndEffectorCommand cmd;
  DexterousHandMessage *message = new DexterousHandMessage ();
  DexterousHandFeedbackMessage *fb_message = new DexterousHandFeedbackMessage ();

  // Constructs the new thread and runs it. Does not block execution.
  std::atomic<bool> query_thread_run_;
  query_thread_run_ = true;
  std::thread query_hand_thread{[&]() {
    // query data
    bool ret = false;
    while (query_thread_run_.load ())
    {
      std::this_thread::sleep_for (std::chrono::milliseconds (5));
      monitor_lock_.lock();
      ret = ee_obj.read_hand_status (message);
      monitor_lock_.unlock();
      if (ret)
      {
        message->Clear (); 
      }
    }
  }};
  query_hand_thread.detach ();

  std::atomic<bool> monitor_thread_run;
  std::function<void()> monitor_thread_func;
  std::thread monitor_thread = std::thread(monitor_thread_func = [&]()->void{
    bool ret = false;
    std::string log_str;
    std::string log_file_path = "../log/log.txt";
    std::ofstream log_file_out;
    
    while (monitor_thread_run.load ())
    {
      std::this_thread::sleep_for (std::chrono::milliseconds (100));

      int timeout = 0;
      DexterousHandFeedbackMessage* message = new DexterousHandFeedbackMessage();
      monitor_lock_.lock();
      ee_obj.hand_read_hand_log();
      while(!ee_obj.read_hand_feedback (message))
      {
          std::this_thread::sleep_for(std::chrono::milliseconds (20));
          timeout++;
          if(timeout > 20)
          {
              monitor_lock_.unlock();
              return;
          }
      }
      monitor_lock_.unlock();

      time_t timep;
      time (&timep);
      char timestamp[64];
      strftime( timestamp, sizeof(timestamp), "%Y/%m/%d %X : ",localtime(&timep));
      log_str = timestamp;
      log_str += message->cmd_param_string();
      std::cout<<log_str<<std::endl;

      if(!log_str.empty())
      {   
          log_file_out.open(log_file_path,std::ios::app);
          log_file_out << log_str << std::endl;
          log_file_out.close();
      }
    }        
  });
  monitor_thread.detach ();

  int input_command = 0;
  std::string choose_command;
  while (true)
  {
    choose_command = command_map[input_command];
    if (!choose_command.compare("command_clear_screen"))
    {
      printf ("\033[2J"); // clean screen
      printf ("\033[H");  // cursor reset
      std::cout << std::endl
                << std::endl
                << "Enter your choice to control the hand :" << std::endl;
      for(int index = 0;index < (int)command_map.size();index++)
      {
        std::cout<< index << " : " <<command_map[index] << std::endl;
      }
    }
    else if (!choose_command.compare("command_read_main_version"))
    {
      monitor_lock_.lock();
      ee_obj.hand_read_main_version();
      std::this_thread::sleep_for(std::chrono::milliseconds (100));
      ee_obj.read_hand_feedback (fb_message);
      if (fb_message->cmd () == DexterousHandFeedbackMessage_Cmd_Type_QUERY_SOFTVERION)
      {
        std::cout << "The hand main version is " << fb_message->cmd_param_string () << std::endl;
      }
      monitor_lock_.unlock();
    }
    else if (!choose_command.compare("command_config_local_ip_address"))
    {   
      std::string input;
      std::cout<<"please input ip address:";
      std::cin>> input;
      std::cout<< "input ip address is: "<< input << std::endl;

      monitor_lock_.lock();
      rst = ee_obj.hand_config_local_ip_address(input);
      if(-1 == rst)
      {
        std::cout << "input ip address error!" << std::endl;
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ee_obj.read_hand_feedback(fb_message);
        if(fb_message->cmd() == DexterousHandFeedbackMessage_Cmd_Type_SET_LOCAL_IP_ADDRESS)
        {
          std::cout << "The FeedbackMessage is " << fb_message->cmd_param_int() << std::endl;
          if(0 == fb_message->cmd_param_int())
          {
            /* update config.json */
            /* read config.json */
            nlohmann::json config_sercan;
            std::ifstream input_sercan(CONFIG_PATH);
            input_sercan >> config_sercan;
            /* find object */
            nlohmann::json &obj_netinf = config_sercan.at("netinf");
            nlohmann::json &obj_socket_map = config_sercan.at("socket_map");
            nlohmann::json &obj_socket_device = config_sercan.at("socket_device");
            /* update value */
            obj_netinf.at("device_ip") = input;
            std::string socket_info = "tcp:" + input + ":2020";
            obj_socket_map.at("41") = socket_info;
            obj_socket_map.at("54") = socket_info;
            obj_socket_map.at("55") = socket_info;
            obj_socket_map.at("56") = socket_info;
            obj_socket_map.at("57") = socket_info;
            obj_socket_map.at("59") = socket_info;
            obj_socket_map.at("60") = socket_info;
            obj_socket_device[0] = socket_info;
            /* rewrite config.json */
            std::ofstream output_sercan(CONFIG_PATH);
            output_sercan.clear();
            output_sercan << config_sercan.dump(4 , ' ', false);
          }
        }
      }
      monitor_lock_.unlock();
    }
    else if(!choose_command.compare("command_config_local_ip_netmask"))
    {
      std::string input;
      std::cout<<"please input netmask:";
      std::cin>> input;
      std::cout<< "input netmask is: "<< input << std::endl;
    
      monitor_lock_.lock();
      rst = ee_obj.hand_config_local_ip_netmask(input);
      if(-1 == rst)
      {
        std::cout << "input netmask error!" << std::endl;
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ee_obj.read_hand_feedback(fb_message);
        if(fb_message->cmd() == DexterousHandFeedbackMessage_Cmd_Type_SET_LOCAL_IP_NETMASK)
        {
          std::cout << "The FeedbackMessage is " << fb_message->cmd_param_int() << std::endl;
          if(0 == fb_message->cmd_param_int())
          {
            /* update config.json */
            /* read config.json */
            nlohmann::json config_sercan;
            std::ifstream input_sercan(CONFIG_PATH);
            input_sercan >> config_sercan;
            /* find object */
            nlohmann::json &obj_netinf = config_sercan.at("netinf");
            /* update value */
            obj_netinf.at("netmask") = input;       
            /* rewrite config.json */
            std::ofstream output_sercan(CONFIG_PATH);
            output_sercan.clear();
            output_sercan << config_sercan.dump(4 , ' ', false);
          }
        }
      } 
      monitor_lock_.unlock();
    }
    else if (!choose_command.compare("command_config_local_ip_gateway"))
    {
      std::string input;
      std::cout<<"please input gateway:";
      std::cin>> input;
      std::cout<< "input gateway is: "<< input << std::endl;

      monitor_lock_.lock();
      rst = ee_obj.hand_config_local_ip_gateway(input);
      if(-1 == rst)
      {
        std::cout << "input gateway error!" << std::endl;
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ee_obj.read_hand_feedback(fb_message);
        if(fb_message->cmd() == DexterousHandFeedbackMessage_Cmd_Type_SET_LOCAL_IP_GATEWAY)
        {
          std::cout << "The FeedbackMessage is " << fb_message->cmd_param_int() << std::endl;
          if(0 == fb_message->cmd_param_int())
          {
            /* update config.json */
            /* read config.json */
            nlohmann::json config_sercan;
            std::ifstream input_sercan(CONFIG_PATH);
            input_sercan >> config_sercan;
            /* find object */
            nlohmann::json &obj_netinf = config_sercan.at("netinf");
            /* update value */
            obj_netinf.at("gateway") = input;       
            /* rewrite config.json */
            std::ofstream output_sercan(CONFIG_PATH);
            output_sercan.clear();
            output_sercan << config_sercan.dump(4 , ' ', false);
          }
        }
      }
      monitor_lock_.unlock();
    }
    else if (!choose_command.compare("command_read_finger_fw_version"))
    {
      int id;
      std::cout<<"enter finger id:";
      std::cin>>id;
      monitor_lock_.lock();
      ee_obj.hand_read_finger_version((finger_id_t)id);
      std::this_thread::sleep_for(std::chrono::milliseconds (100));
      ee_obj.read_hand_feedback (fb_message);
      if (fb_message->cmd () == DexterousHandFeedbackMessage_Cmd_Type_QUERY_FINGER_SOFTVERSION)
      {
        std::cout << "The hand finger version is " << fb_message->cmd_param_string () << std::endl;
      }
      monitor_lock_.unlock();
    }
    else if (!choose_command.compare("command_read_finger_config_version"))
    {
      int id;
      std::cout<<"enter finger id:";
      std::cin>>id;
      monitor_lock_.lock();
      ee_obj.hand_read_finger_config_version((finger_id_t)id);
      std::this_thread::sleep_for(std::chrono::milliseconds (100));
      ee_obj.read_hand_feedback (fb_message);
      if (fb_message->cmd () == DexterousHandFeedbackMessage_Cmd_Type_QUERY_FINGER_CONFIG_VERSION)
      {
        std::cout << "The hand finger version is " << fb_message->cmd_param_string () << std::endl;
      }
      monitor_lock_.unlock();
    }
    else if (!choose_command.compare("command_read_product_type"))
    {
      monitor_lock_.lock();
      ee_obj.hand_read_product_type();
      std::this_thread::sleep_for(std::chrono::milliseconds (100));
      ee_obj.read_hand_feedback (fb_message);
      if (fb_message->cmd () == DexterousHandFeedbackMessage_Cmd_Type_QUERY_HAND_PRODUCT_TYPE)
      {
        std::cout << "The hand product id is " << fb_message->cmd_param_int () << std::endl;
      }
      monitor_lock_.unlock();
    }
    else if (!choose_command.compare("command_read_report_rate"))
    {
        std::cout << "The hand report rate id is " << ee_obj.get_hand_report_rate() << std::endl;
    }
    else if (!choose_command.compare("command_reset_hand"))
    {
        ee_obj.hand_reset ();
    }
    else if (!choose_command.compare("command_release_hand"))
    {
        ee_obj.hand_release ();
    }
    else if (!choose_command.compare("command_stop_hand"))
    {
        ee_obj.hand_stop ();
    }
    else if (!choose_command.compare("command_paralle_change_posture"))
    {
        int gesture;
        std::cout << "please input the gesture type:(0-three, 1-two, 2-two_opposite)";
        std::cin >> gesture;
        ee_obj.hand_change_paralle_posture (gesture);
    }
    else if (!choose_command.compare("command_paralle_grasp_distance"))
    {
        int gesture;
        float input_distance;

        std::cout << "please input the gesture type(0-three, 1-two, 2-two_opposite): ";
        std::cin >> gesture;
        std::cout << "please input the distance in float type:(no exceed 200mm): ";
        std::cin >> input_distance;
        ee_obj.hand_paralle_grasp_distance (gesture, input_distance);
    }
    else if (!choose_command.compare("command_paralle_grasp_current"))
    {
        int gesture;
        float input_current_0;
        float input_current_1;

        std::cout << "please input the gesture type(0-three, 1-two, 2-two_opposite): ";
        std::cin >> gesture;
        std::cout << "please input the current in float type:(no exceed 700mA): ";
        std::cin >> input_current_0;
        std::cout << "please input the current in float type:(no exceed 400mA): ";
        std::cin >> input_current_1;
        ee_obj.hand_paralle_grasp_current (gesture, input_current_0, input_current_1);
    }
    else if (!choose_command.compare("command_control_position"))
    {
        DexterousHand_JointAction joint;
        float control_position;
        DexterousHand_FingerCommand finger;
        DexterousHandCommand hand;

        int control_finger;
        int control_demand;

add_figner_position:

        std::cout << "please input the control finger id: ";
        std::cin >> control_finger;

        if(control_finger < 7) 
        {
          std::cout << "please input the joint_1 control_position in float type: ";
          std::cin >> control_position;
          if (std::abs(control_position) <= 90)
          {
            joint.joint_action = dr::joint_control_t::CONTROL_POSITION;
            joint.position = control_position;
            finger.joints[ee_obj.choose_joint(JOINT_ID_1)] = joint;
            hand.fingers[ee_obj.choose_finger((finger_id_t)control_finger)] = finger;
          }

          std::cout << "please input the joint_0 control_position in float type: ";
          std::cin >> control_position;
          if (std::abs(control_position) <= 90)
          {
            joint.joint_action = dr::joint_control_t::CONTROL_POSITION;
            joint.position = control_position;
            finger.joints[ee_obj.choose_joint(JOINT_ID_0)] = joint;
            hand.fingers[ee_obj.choose_finger((finger_id_t)control_finger)] = finger;
          }

          std::cout << "Do you want to add finger or change finger: 0, no thanks; 1, yes, i want to :";
          std::cin >> control_demand;
          if(control_demand == 1)
          {
            goto add_figner_position;
          }

          cmd.dexterous_hand_command = hand;
          cmd.dexterous_hand_command.control_type = dr::hand_control_t::SYNC_CONTROL;

          ee_obj.execute (cmd);
        }
    }
    else if (!choose_command.compare("command_control_current"))
    {
        DexterousHand_JointAction joint;
        float control_current;
        DexterousHand_FingerCommand finger;
        DexterousHandCommand hand;

        int control_finger;
        int control_demand;

add_figner_current:

        std::cout << "please input the control finger id:";
        std::cin >> control_finger;

        if(control_finger == 0)
        {
          std::cout << "palm joint not support current control!" << std::endl;
        }
        else if(control_finger < 7) 
        {
          std::cout << "please input the joint_1 control_current in float type(not exceed 400): ";
          std::cin >> control_current;
          if (control_current <= 400)
          {
            joint.joint_action = dr::joint_control_t::CONTROL_CURRENT;
            joint.current = control_current;
            finger.joints[ee_obj.choose_joint(JOINT_ID_1)] = joint;
            hand.fingers[ee_obj.choose_finger((finger_id_t)control_finger)] = finger;
          }

          std::cout << "please input the joint_0 control_current in float type(not exceed 700): ";
          std::cin >> control_current;
          if (control_current <= 700)
          {
            joint.joint_action = dr::joint_control_t::CONTROL_CURRENT;
            joint.current = control_current;
            finger.joints[ee_obj.choose_joint(JOINT_ID_0)] = joint;
            hand.fingers[ee_obj.choose_finger((finger_id_t)control_finger)] = finger;
          }

          std::cout << "Do you want to add finger or change finger: 0, no thanks; 1, yes, i want to :";
          std::cin >> control_demand;
          if(control_demand == 1)
          {
            goto add_figner_current;
          }

          cmd.dexterous_hand_command = hand;
          cmd.dexterous_hand_command.control_type = dr::hand_control_t::SYNC_CONTROL;

          ee_obj.execute (cmd);
        }
    }
    else if (!choose_command.compare("command_control_force"))
    {
        DexterousHand_JointAction joint;
        float control_force;
        DexterousHand_FingerCommand finger;
        DexterousHandCommand hand;

        int control_finger;
        int control_demand;

add_figner_force:

        std::cout << "please input the control finger id: ";
        std::cin >> control_finger;
        if(control_finger == 0)
        {
          std::cout << "palm joint not support force control!" << std::endl;
        }
        else if(control_finger < 7) 
        {
          std::cout << "please input the joint_1 control_force in float type(not exceed 3000): ";
          std::cin >> control_force;
          if (control_force < 3000)
          {
            joint.joint_action = dr::joint_control_t::CONTROL_FORCE;
            joint.force = control_force;
            finger.joints[ee_obj.choose_joint(JOINT_ID_1)] = joint;
            hand.fingers[ee_obj.choose_finger((finger_id_t)control_finger)] = finger;
          }

          std::cout << "please input the joint_0 control_force in float type(not exceed 3000): ";
          std::cin >> control_force;
          if (control_force < 3000)
          {
            joint.joint_action = dr::joint_control_t::CONTROL_FORCE;
            joint.force = control_force;
            finger.joints[ee_obj.choose_joint(JOINT_ID_0)] = joint;
            hand.fingers[ee_obj.choose_finger((finger_id_t)control_finger)] = finger;
          }

          std::cout << "Do you want to add finger or change finger: 0, no thanks; 1, yes, i want to :";
          std::cin >> control_demand;
          if(control_demand == 1)
          {
            goto add_figner_force;
          }

          cmd.dexterous_hand_command = hand;
          cmd.dexterous_hand_command.control_type = dr::hand_control_t::SYNC_CONTROL;

          ee_obj.execute (cmd);
        }
    }
    else if (!choose_command.compare("command_control_individual_3f"))
    {
      int mode;
      float input_value_0 = 9999;
      float input_value_1 = 9999;
      int control_flag = 0;
      std::vector<float> target {0,0,9999,9999,9999,9999,9999,9999,9999,9999};

      std::cout << "please input the control mode type(0-position, 1-current, 2-velocity, 3-force): ";
      std::cin >> mode;

      if(mode >= 0 && mode <= 3)
      {
        target[0] = mode;
      }
      else
      {
        break;
      }

      for(uint i = 1; i < 5; i++)
      {
        std::cout << "Do you want to control target " << i - 1 << " (0-no, 1-yes): ";
        std::cin >> control_flag;

        if (control_flag != 0)
        {
          std::cout << "please input the value in joint 0 (9999-no motion): ";
          std::cin >> input_value_0;
          std::cout << "please input the value in joint 1 (9999-no motion): ";
          std::cin >> input_value_1;
          if (input_value_0 != 9999)
          {
            target[i * 2] = input_value_0;
          }
          if (input_value_0 != 9999)
          {
            target[i * 2 + 1] = input_value_1;
          }
        }
      }

      ee_obj.hand_joint_control(target);
    }
    else if (!choose_command.compare("command_control_individual_5f"))
    {
      int mode;
      float input_value_0 = 9999;
      float input_value_1 = 9999;
      int control_flag = 0;
      std::vector<float> target {0,1,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999};

      std::cout << "please input the control mode type(0-position, 1-current, 2-velocity, 3-force): ";
      std::cin >> mode;

      if(mode >= 0 && mode <= 3)
      {
        target[0] = mode;
      }
      else
      {
        break;
      }

      for(uint i = 1; i < 8; i++)
      {
        std::cout << "Do you want to control target " << i - 1 << " (0-no, 1-yes): ";
        std::cin >> control_flag;

        if (control_flag != 0)
        {
          std::cout << "please input the value in joint 0 (9999-no motion): ";
          std::cin >> input_value_0;
          std::cout << "please input the value in joint 1 (9999-no motion): ";
          std::cin >> input_value_1;
          if (input_value_0 != 9999)
          {
            target[i * 2] = input_value_0;
          }
          if (input_value_0 != 9999)
          {
            target[i * 2 + 1] = input_value_1;
          }
        }
      }

      ee_obj.hand_joint_control(target);
    }
    else if (!choose_command.compare("command_set_max_current"))
    {
      int input_finger_id;
      std::cout << "please input the control finger id: ";
      std::cin >> input_finger_id;

      int input_joint_id;
      std::cout << "please input the control joint id: ";
      std::cin >> input_joint_id;

      float input_max_current;
      std::cout << "please input the current in float typ(for joint-1 no exceed 400mA,for joint-0 no exceed 700mA): ";
      std::cin >> input_max_current; 

      ee_obj.hand_set_max_current((finger_id_t)input_finger_id,(joint_id_t)input_joint_id,input_max_current);
    }
    else if (!choose_command.compare("command_set_speed"))
    {
      int input_finger_id;
      std::cout << "please input the control finger id:";
      std::cin >> input_finger_id;

      int input_joint_id;
      std::cout << "please input the control joint id:";
      std::cin >> input_joint_id;

      float input_speed;
      std::cout << "please input the speed in float type(unit: deg/s): ";
      std::cin >> input_speed; 

      ee_obj.hand_set_speed((finger_id_t)input_finger_id,(joint_id_t)input_joint_id,input_speed);
    }
    else if (!choose_command.compare("command_set_report_rate"))
    {
      float input_report_rate;
      std::cout << "please input the input_report_rate(not exceed 60HZ): ";
      std::cin >> input_report_rate;

      if(input_report_rate <= 60)
        ee_obj.set_report_rate(input_report_rate);
    }
    else if (!choose_command.compare("command_control_5F_diff_joint"))
    {
      int input_control_type;
      std::cout << "please input the input_control_type:0-current,1-position-1,2-position-2: ";
      std::cin >> input_control_type;

      if(input_control_type == 1)
      {
        float angle_pj,angle_mj;
        std::cout << "please input angle_pj(-60~150)";
        std::cin >> angle_pj;
        std::cout << "please input angle_mj(-60~150)";
        std::cin >> angle_mj;

        finger_id_t finger_id[2] = {PALM_ID_0,PALM_ID_0};
        joint_id_t joint_id[2] = {JOINT_ID_0,JOINT_ID_1};
        
        DexterousHand_JointAction pj;
        DexterousHand_JointAction mj;

        DexterousHand_FingerCommand finger;
        pj.joint_action = dr::joint_control_t::CONTROL_POSITION;
        pj.position = angle_pj;
        finger.joints["joint_1"] = pj;
        mj.joint_action = dr::joint_control_t::CONTROL_POSITION;
        mj.position = angle_mj;
        finger.joints["joint_0"] = mj;

        DexterousHandCommand hand;
        hand.fingers["palm_0"] = finger;
        cmd.dexterous_hand_command = hand;

        ee_obj.execute (cmd);
      }
      else if(input_control_type == 0)
      {
        int control_type;
        std::cout << "please input control type: 0 left, 1 right, 2 down, 3 up: ";
        std::cin >> control_type;
        float control_current;
        std::cout << "please input control current: ";
        std::cin >> control_current;

        finger_id_t finger_id[2] = {PALM_ID_0,PALM_ID_0};
        joint_id_t joint_id[2] = {JOINT_ID_0,JOINT_ID_1};
        
        if(0 == control_type) // left
        {
          float current[2] = {0-control_current,0-control_current};
          ee_obj.hand_set_current(2,finger_id,joint_id,current);
        }
        else if(1 == control_type)
        {
          float current[2] = {control_current,control_current}; // right
          ee_obj.hand_set_current(2,finger_id,joint_id,current);
        }
        else if(2 == control_type)
        {
          float current[2] = {control_current,0-control_current}; // down
          ee_obj.hand_set_current(2,finger_id,joint_id,current);
        }
        else if(3 == control_type)
        {
          float current[2] = {0-control_current,control_current}; // up
          ee_obj.hand_set_current(2,finger_id,joint_id,current);
        }
      }
      else if(input_control_type == 2)
      {
        float angle_0,angle_1;
        std::cout << "please input angle_0(0~90)";
        std::cin >> angle_0;
        std::cout << "please input angle_1(-30~30)";
        std::cin >> angle_1;

        finger_id_t finger_id[2] = {PALM_ID_0,PALM_ID_0};
        joint_id_t joint_id[2] = {JOINT_ID_0,JOINT_ID_1};
        
        DexterousHand_JointAction pj;
        DexterousHand_JointAction mj;

        DexterousHand_FingerCommand finger;
        pj.joint_action = dr::joint_control_t::CONTROL_POSITION;
        pj.position = angle_0 + 2 * angle_1;
        finger.joints["joint_1"] = pj;
        mj.joint_action = dr::joint_control_t::CONTROL_POSITION;
        mj.position = angle_0 - 2 * angle_1;
        finger.joints["joint_0"] = mj;

        DexterousHandCommand hand;
        hand.fingers["palm_0"] = finger;
        cmd.dexterous_hand_command = hand;

        ee_obj.execute (cmd);
      }
    }
    else if (!choose_command.compare("command_test_case_1"))
    {
      float test_position = 0;
      float test_position_dir = 0;
      while(1)
      {
        std::this_thread::sleep_for (std::chrono::milliseconds (40));

        finger_id_t finger_id[5] = {FINGER_ID_1,FINGER_ID_2,FINGER_ID_3,PALM_ID_0,PALM_ID_0};
        joint_id_t joint_id[5] = {JOINT_ID_0,JOINT_ID_0,JOINT_ID_0,JOINT_ID_0,JOINT_ID_1};
        float position[5] = {0};
        if(test_position_dir)
        {
          if(test_position < 90)
          {
            test_position += 5;
          }
          else
          {
            test_position_dir = 0;
          }
        }
        else
        {
          if(test_position > -90)
          {
            test_position -= 5;
          }
          else
          {
            test_position_dir = 1;
          }
        }
        
        position[0] = test_position;
        position[1] = test_position;
        position[2] = test_position;
        position[3] = 0;
        position[4] = 0;
        ee_obj.hand_move_finger(5,finger_id,joint_id,position);
      }
      break;
    }
    else if (!choose_command.compare("command_rtkp_get"))
    {
      int id;
      std::cout<<"enter finger id:";
      std::cin>>id;
      ee_obj.hand_rtkp_get((finger_id_t)id);
      while(!ee_obj.read_hand_feedback (fb_message));
      if (fb_message->cmd () == DexterousHandFeedbackMessage_Cmd_Type_QUERY_RUNTIME_KEY_PARAMETER)
      {
        // std::cout << "command_rtkp_get! " << std::endl;
        // std::vector<char> serial_byte(fb_message->cmd_param_buffer().begin(), fb_message->cmd_param_buffer().end());
        // serial_byte.push_back('\0');
        // std::cout << "recevied data:" << std::endl;
        // for(uint i=0; i<serial_byte.size(); ++i)
        //   std::cout << std::hex << std::to_string(serial_byte[i]) << std::hex << ' ';
        // std::cout << std::endl;

        dexterous_finger::rt_key_parameter kp_message;
        kp_message.Clear();
        kp_message.ParseFromString (fb_message->cmd_param_buffer());
        std::cout << kp_message.flash_sector() << std::endl;
        std::cout << kp_message.record_count() << std::endl;
        std::cout << kp_message.can_error_count() << std::endl;
        std::cout << kp_message.max_temperature(0) << std::endl;
      }
    }
    else if (!choose_command.compare("command_rtkp_record"))
    {
      int id;
      std::cout<<"enter finger id:";
      std::cin>>id;
      ee_obj.hand_rtkp_record((finger_id_t)id);
      while(!ee_obj.read_hand_feedback (fb_message));
      if (fb_message->cmd () == DexterousHandFeedbackMessage_Cmd_Type_RECORD_RUNTIME_KEY_PARAMETER)
      {
        std::cout << "command_rtkp_record! " << std::endl;
      }
    }
    else if (!choose_command.compare("command_rtkp_clear"))
    {
      int id;
      std::cout<<"enter finger id:";
      std::cin>>id;
      ee_obj.hand_rtkp_clear((finger_id_t)id);
      while(!ee_obj.read_hand_feedback (fb_message));
      if (fb_message->cmd () == DexterousHandFeedbackMessage_Cmd_Type_CLEAR_RUNTIME_KEY_PARAMETER)
      {
        std::cout << "command_rtkp_clear! " << std::endl;
      }
    }
    std::cout << "Enter Number:";
    std::cin >> input_command;
  }

  query_thread_run_ = false;

  return 0;
}

void change_interface(int type)
{
    nlohmann::json config_sercan;
    std::ifstream input_sercan(CONFIG_PATH);
    input_sercan >> config_sercan;
    std::string comm_method = config_sercan.at("comm_method");

    switch (type)
    {
    case 1: /*usb*/
       std::cout <<"select USB" << std::endl;
      config_sercan.at("comm_method") = "serial";
      break;
    case 2: /*ethernet*/
       std::cout <<"select ETHERNET" << std::endl;
      config_sercan.at("comm_method") = "socket";
      break;
    }

    comm_method = config_sercan.at("comm_method");
    std::ofstream output_sercan(CONFIG_PATH);
    output_sercan.clear();
    output_sercan << config_sercan.dump(4 , ' ', false);
}

/* Display program usage, and exit.
 */
static void print_dexterous_usage( FILE* stream, int exit_code )
{
  fprintf (stream, "Usage: %s [options] \n\n", dexterous_program_name);
  fprintf (
      stream, 
      "Options:\n\r"
      " -h --help .\n"
      " -R --resetip [host ip].                    reset doraHand ip to default.\n"
      " -i --interface=<type>.                     type refers to eth, usb.\n"
      "\n"    );
  exit (exit_code);
}

void dexterous_args_update( int argc, char **argv)
{
  IO_SERCAN_HAND *ptr_io;
  int next_option;
  const char* const short_options = "hR:";    // list of short options
  const struct option long_options[] =         // structure of long options
  {
    { "help", 0, NULL, 'h' },
    { "resetip",1, NULL, 'R' },
    { "interface",1, NULL, 'i' },
    { NULL, 0, NULL, 0}
  };

  const char* output_filename = NULL;
  dexterous_program_name = argv[0];

  /* Process the arguments with getopt_long(), then 
	 * populate globalArgs. 
	 */
	do
  {
    next_option = getopt_long (argc, argv, short_options, long_options, NULL);
    switch (next_option)
    {
        case 'h':     		//-h or --help, print the info of help   
        case '?':  
          print_dexterous_usage (stdout, 0);
          break;
        case 'R':   		//-R or    --resetip
          ptr_io->hand_reset_local_ip(optarg);
          exit(0);
          break;
        case 'i':   		//-i or    --interface
                if(0 == strcmp(optarg,"usb"))
                {
                    change_interface(1);
                }
                else if(0 == strcmp(optarg,"eth"))
                {
                    change_interface(2);
                }
                else
                {
                    printf("interface type not support!\n");
                    exit(0);
                }
                break;
        default:
          break;
    }
  }while (next_option !=-1);
}

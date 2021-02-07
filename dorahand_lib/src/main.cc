/**
 * @file main.cc
 * @author Dorabot Inc.
 * @brief library demo for control DoraHand
 * @version 1.1.1
 * @date 2021-02-08
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <dorahand.hh>
#include <atomic>
#include <chrono>
#include <mutex>
#include <time.h>
#include <unistd.h>
#include <iostream>
// the time duration of every joint command should longer than 50ms

using namespace dr;

std::vector<std::string> command_name =
{
  "command_clear_screen",
  "command_stop_hand",
  "command_reset_hand",
  "command_release_hand",
  "command_read_main_version",
  "command_read_finger_version",
  "command_read_product_type",
  "command_read_report_rate",
  "command_read_sensor_data",
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
std::map<int,std::string> command_map;
void command_map_init()
{ 
  command_map.clear();
  for(uint command_index=0;command_index<command_name.size();command_index++)
  {
    command_map.insert(std::pair<int,std::string>(command_index,command_name[command_index]));
  }
} 

int main (int argc, char **argv)
{
  command_map_init();
  
  IO_SERCAN_HAND ee_obj;

  if(!ee_obj.on_init());
  {
    exit;
  }
  ee_obj.set_callback();
  ee_obj.on_monitor();

  std::mutex locker_;
  std::vector<DexterousHandMessage> message_queue;
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
      ret = ee_obj.read_hand_status (message);
      if (ret)
      {
        
        message->Clear (); 
      }
    }
  }};
  query_hand_thread.detach ();

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
      ee_obj.hand_read_main_version();
      ee_obj.read_hand_feedback (fb_message);
      usleep(100000);
      if (fb_message->cmd () == DexterousHandFeedbackMessage_Cmd_Type_QUERY_SOFTVERION)
      {
        std::cout << "The hand main version is " << fb_message->cmd_param_string () << std::endl;
      }
    }
    else if (!choose_command.compare("command_read_sensor_data"))
    {
      ee_obj.hand_read_main_version();
      ee_obj.read_hand_feedback (fb_message);
      usleep(100000);
      if (fb_message->cmd () == DexterousHandFeedbackMessage_Cmd_Type_QUERY_SOFTVERION)
      {
        std::cout << "The hand main version is " << fb_message->cmd_param_string () << std::endl;
      }
    }
    else if (!choose_command.compare("command_read_finger_version"))
    {
      int id;
      std::cout<<"enter finger id:";
      std::cin>>id;
      ee_obj.hand_read_finger_version((finger_id_t)id);
      ee_obj.read_hand_feedback (fb_message);
      usleep(100000);
      if (fb_message->cmd () == DexterousHandFeedbackMessage_Cmd_Type_QUERY_FINGER_SOFTVERSION)
      {
        std::cout << "The hand finger version is " << fb_message->cmd_param_string () << std::endl;
      }
    }
    else if (!choose_command.compare("command_read_product_type"))
    {
      ee_obj.hand_read_product_type();
      ee_obj.read_hand_feedback (fb_message);
      usleep(100000);
      if (fb_message->cmd () == DexterousHandFeedbackMessage_Cmd_Type_QUERY_HAND_PRODUCT_TYPE)
      {
        std::cout << "The hand product id is " << fb_message->cmd_param_int () << std::endl;
      }
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
        std::cout << "please input the distance in float type:(no exceed 100mm): ";
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
        std::cout << "please input the current in float type:(no exceed 400mA): ";
        std::cin >> input_current_0;
        std::cout << "please input the current in float type:(no exceed 200mA): ";
        std::cin >> input_current_1;
        ee_obj.hand_paralle_grasp_current (gesture, input_current_0, input_current_1);
    }
    else if (!choose_command.compare("command_control_force"))
    {
        DexterousHand_JointAction joint;
        float control_force;
        DexterousHand_FingerCommand finger;
        DexterousHandCommand hand;

        int control_finger;
        std::cout << "please input the control finger id: ";
        std::cin >> control_finger;
        if(control_finger < 7) 
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

          cmd.dexterous_hand_command = hand;

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

add_figner:

        std::cout << "please input the control finger id:";
        std::cin >> control_finger;
        if(control_finger < 7) 
        {
          std::cout << "please input the joint_1 control_current in float type(not exceed 1000): ";
          std::cin >> control_current;
          if (control_current < 1000)
          {
            joint.joint_action = dr::joint_control_t::CONTROL_CURRENT;
            joint.current = control_current;
            finger.joints[ee_obj.choose_joint(JOINT_ID_1)] = joint;
            hand.fingers[ee_obj.choose_finger((finger_id_t)control_finger)] = finger;
          }

          std::cout << "please input the joint_0 control_current in float type(not exceed 1000): ";
          std::cin >> control_current;
          if (control_current < 1000)
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
            goto add_figner;
          }

          cmd.dexterous_hand_command = hand;

          ee_obj.execute (cmd);
        }
    }
    else if (!choose_command.compare("command_control_position"))
    {
        DexterousHand_JointAction joint;
        float control_position;
        DexterousHand_FingerCommand finger;
        DexterousHandCommand hand;

        int control_finger;
        std::cout << "please input the control finger id: ";
        std::cin >> control_finger;
        if(control_finger < 7) 
        {
          std::cout << "please input the joint_1 control_position in float type: ";
          std::cin >> control_position;
          if (control_position < 180)
          {
            joint.joint_action = dr::joint_control_t::CONTROL_POSITION;
            joint.position = control_position;
            finger.joints[ee_obj.choose_joint(JOINT_ID_1)] = joint;
            hand.fingers[ee_obj.choose_finger((finger_id_t)control_finger)] = finger;
          }

          std::cout << "please input the joint_0 control_position in float type: ";
          std::cin >> control_position;
          if (control_position < 180)
          {
            joint.joint_action = dr::joint_control_t::CONTROL_POSITION;
            joint.position = control_position;
            finger.joints[ee_obj.choose_joint(JOINT_ID_0)] = joint;
            hand.fingers[ee_obj.choose_finger((finger_id_t)control_finger)] = finger;
          }

          cmd.dexterous_hand_command = hand;

          ee_obj.execute (cmd);
        }
    }
    else if (!choose_command.compare("command_control_individual"))
    {
      int mode;
      float input_value_0 = 999;
      float input_value_1 = 999;
      int control_flag = 0;
      std::vector<float> target {0,0,999,999,999,999,999,999,999,999};

      std::cout << "please input the control mode type(0-position, 1-current, 2-velocity, 3-force): ";
      std::cin >> mode;

      for(uint i = 1; i < 5; i++)
      {
        std::cout << "Do you want to control target(0-no, 1-yes):" << i - 1;
        std::cin >> control_flag;

        if (control_flag != 0)
        {
          std::cout << "please input the value in joint 0 (999-no motion): ";
          std::cin >> input_value_0;
          std::cout << "please input the value in joint 0 (999-no motion): ";
          std::cin >> input_value_1;
          if (input_value_0 != 999)
          {
            target[i * 2] = input_value_0;
          }
          if (input_value_0 != 999)
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
      std::cout << "please input the current in float typ(for joint-1 no exceed 250mA,for joint-0 no exceed 550mA): ";
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
    std::cout << "Enter Number:";
    std::cin >> input_command;
  }

  query_thread_run_ = false;

  return 0;
}

cmake_minimum_required(VERSION 3.5)
project(dorahand_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  message_generation
  urdf
)

find_package(Threads)
find_package(Protobuf REQUIRED)
find_package(urdf REQUIRED)
find_package(dorahand REQUIRED)

add_service_files(
  FILES
  ControlJoint.srv
  GetHandState.srv
  GraspControl.srv
  SetHandState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
  /usr/include/dorahand
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  dorahand_ros
)

add_executable(dorahand_ros src/dorahand_ros.cpp)
target_link_libraries(dorahand_ros
  ${catkin_LIBRARIES}
)
target_link_libraries(dorahand_ros dorahand)
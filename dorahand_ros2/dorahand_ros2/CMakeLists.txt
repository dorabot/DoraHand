cmake_minimum_required(VERSION 3.5)
project(dorahand_ros2)

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

include_directories(include)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${dorahand_INCLUDE_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Threads)
find_package(Protobuf 3.6.1 REQUIRED)
find_package(dorahand REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(dorahand_interfaces_ros2 REQUIRED)

set(SOURCE
src/dorahand_ros2.cpp
src/dorahand_ros2_node.cc
)

add_executable(dorahand_ros2 ${SOURCE})
target_include_directories(dorahand_ros2 PUBLIC include)
target_link_libraries(${PROJECT_NAME} dorahand ${PROTOBUF_LIBRARIES} ${dorahand_LIBRARIES})
ament_target_dependencies(dorahand_ros2 dorahand dorahand_interfaces_ros2 rclcpp std_msgs std_srvs urdf ament_index_cpp)

install(TARGETS dorahand_ros2
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
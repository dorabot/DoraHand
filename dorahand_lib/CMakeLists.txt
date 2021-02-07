project(demo_call_lib)

# cmake version
cmake_minimum_required(VERSION 3.7)

# find package and link
find_package(Threads)
find_package(Protobuf REQUIRED)
find_package(dorahand REQUIRED)

# add include
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(/usr/include/dorahand)

# scan all source file
aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_LIST)

# build target
add_executable(demo_call_lib ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} dorahand)
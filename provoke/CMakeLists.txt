cmake_minimum_required(VERSION 3.5)
project(provoke)

set(provoke_MAJOR_VERSION 0)
set(provoke_MINOR_VERSION 0)
set(provoke_PATCH_VERSION 0)
set(provoke_VERSION
  ${provoke_MAJOR_VERSION}.${provoke_MINOR_VERSION}.${provoke_PATCH_VERSION})

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


# Find packages
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(tello_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Local includes
include_directories(include
  ${geometry_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${ros2_shared_INCLUDE_DIRS}
  ${tello_msgs_INCLUDE_DIRS}
  ${yaml_cpp_vendor_INCLUDE_DIRS}
  )

add_executable(${PROJECT_NAME}
  src/args_interface.cpp
  src/provoke_node_impl.cpp
  src/provoke_node_main.cpp
  src/yaml_args.cpp
  src/result.cpp
  src/timer_machine_par.cpp
  src/timer_machine_pause.cpp
  src/timer_machine_tello.cpp
  src/timer_dispatch.cpp
  src/timer_interface.cpp
  src/base_machine.cpp
  src/tello_dispatch.cpp
  src/tello_interface.cpp
  src/tello_machine_action.cpp
  src/tello_machine_const_action.cpp
  src/tello_machine_send_action.cpp
  )

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  rclcpp
  ros2_shared
  tello_msgs
  yaml_cpp_vendor
  )

#=============
# Install
#=============

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  )

# Install various directories
install(DIRECTORY
  cfg
  launch
  DESTINATION share/${PROJECT_NAME}
  )


ament_package()

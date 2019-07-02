cmake_minimum_required(VERSION 3.5)
project(provoke)

set(provoke_MAJOR_VERSION 0)
set(provoke_MINOR_VERSION 0)
set(provoke_PATCH_VERSION 0)
set(provoke_VERSION
  ${provoke_MAJOR_VERSION}.${provoke_MINOR_VERSION}.${provoke_PATCH_VERSION})

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
find_package(tello_msgs REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(provoke_node
  src/provoke_node.cpp
  src/provoke_node_impl.cpp
  src/sm_manager.cpp
  src/sm_manager_states.cpp
  src/sm_send_action.cpp
  src/sm_send_action_states.cpp
  src/state_machine_interface.cpp
  )

target_include_directories(provoke_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PUBLIC ${tello_msgs_INCLUDE_DIRS}
  PUBLIC ${ros2_shared_INCLUDE_DIRS}
  PUBLIC ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(provoke_node
 ${tello_msgs_LIBRARIES}
 ${ros2_shared_LIBRARIES}
 ${rclcpp_LIBRARIES}
)

install(
  TARGETS provoke_node
  EXPORT export_provoke
  DESTINATION lib/${PROJECT_NAME})

# export targets
set(export_targets ${export_targets};provoke_node)
export(EXPORT export_provoke
  FILE "${PROJECT_BINARY_DIR}/export_provoke.cmake")

# Create the provokeConfig.cmake
set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
configure_file(provokeConfig.cmake.in
  "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/provokeConfig.cmake" @ONLY)

# Create the provokeConfigVersion.cmake
configure_file(provokeConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/provokeConfigVersion.cmake" @ONLY)

install(FILES
  "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/provokeConfig.cmake"
  "${PROJECT_BINARY_DIR}/provokeConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake" COMPONENT dev)
install(EXPORT export_provoke
  DESTINATION "share/${PROJECT_NAME}/cmake"
  FILE export_provoke.cmake
  COMPONENT dev)

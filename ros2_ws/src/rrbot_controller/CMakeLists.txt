cmake_minimum_required(VERSION 3.5)
project(rrbot_controller)

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
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(example_interfaces REQUIRED)

add_library(
  rrbot_controller
  SHARED
  src/rrbot_controller.cpp
)
target_include_directories(
  rrbot_controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  rrbot_controller
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
# prevent pluginlib from using boost
target_compile_definitions(rrbot_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_library(
  rrbot_controller_array
  SHARED
  src/rrbot_controller_array.cpp
)
target_include_directories(
  rrbot_controller_array
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rrbot_controller_array
  control_msgs
  controller_interface
  example_interfaces
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
# prevent pluginlib from using boost
target_compile_definitions(rrbot_controller_array PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


pluginlib_export_plugin_description_file(
  controller_interface rrbot_controller.xml)

install(
  TARGETS
  rrbot_controller rrbot_controller_array
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_rrbot_controller test/test_load_rrbot_controller.cpp)
  target_include_directories(test_load_rrbot_controller PRIVATE include)
  ament_target_dependencies(
    test_load_rrbot_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(test_rrbot_controller test/test_rrbot_controller.cpp)
  target_include_directories(test_rrbot_controller PRIVATE include)
  target_link_libraries(test_rrbot_controller rrbot_controller)
  ament_target_dependencies(
    test_rrbot_controller
    controller_interface
    hardware_interface
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  rrbot_controller
)
ament_export_dependencies(
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

ament_package()

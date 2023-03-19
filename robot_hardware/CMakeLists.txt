cmake_minimum_required(VERSION 3.8)
project(robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_cmake REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/oneDof_system_simple.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

#target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
#target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()

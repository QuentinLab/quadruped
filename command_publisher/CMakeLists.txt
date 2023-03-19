cmake_minimum_required(VERSION 3.8)
project(command_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)


add_executable(pospublisher src/position_publisher.cpp)
ament_target_dependencies(pospublisher rclcpp trajectory_msgs)

add_executable(actionpublisher src/action_joint.cpp)
ament_target_dependencies(actionpublisher rclcpp control_msgs rclcpp_action)

install(TARGETS
    pospublisher
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    actionpublisher 
    DESTINATION lib/${PROJECT_NAME})

ament_package()

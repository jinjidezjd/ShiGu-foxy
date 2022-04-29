cmake_minimum_required(VERSION 3.5)
project(base_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(base_controller src/base_controller.cpp)
ament_target_dependencies(base_controller
  rclcpp
  std_msgs
  geometry_msgs
  serial)

install(TARGETS base_controller
  DESTINATION lib/${PROJECT_NAME})
# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(Mymsg
#   "msg/Mymsg.msg"
# )



ament_package()
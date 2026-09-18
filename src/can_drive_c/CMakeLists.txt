cmake_minimum_required(VERSION 3.8)
project(can_drive_c)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_messages REQUIRED)

add_executable(can_drive_c_node src/can_drive_c_node.cpp)
target_compile_features(can_drive_c_node PUBLIC cxx_std_17)

target_link_libraries(can_drive_c_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${custom_messages_TARGETS}
)

install(TARGETS
  can_drive_c_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

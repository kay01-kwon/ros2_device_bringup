cmake_minimum_required(VERSION 3.8)
project(px4_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)

include_directories(
  include
)

add_executable(px4_client_node
  src/px4_client_node.cpp
  include/px4_launch/px4_client.cpp
)

ament_target_dependencies(px4_client_node
  rclcpp
  mavros_msgs
)

install(TARGETS
  px4_client_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



ament_package()

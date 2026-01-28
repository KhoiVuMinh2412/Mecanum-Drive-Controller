cmake_minimum_required(VERSION 3.8)
project(mecanum_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include/mecanum_driver)

add_library(mecanum_lib
  src/can_comm.cpp
  src/odrivemotor.cpp
  src/mecanum.cpp
)

ament_target_dependencies(mecanum_lib
 rclcpp
 std_msgs
 geometry_msgs
)

add_executable(mecanum_node
  src/mecanum_driver.cpp
)

add_executable(keyboard_node
src/teleop_8_direction.cpp
)

ament_target_dependencies(keyboard_node rclcpp geometry_msgs)

install(TARGETS
  keyboard_node
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(mecanum_node mecanum_lib)
ament_target_dependencies(mecanum_node
  rclcpp
  geometry_msgs
)

install(TARGETS
  mecanum_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(hal_pose_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hal_motor_control_interfaces REQUIRED)
find_package(common_utils REQUIRED)

###########
## Build ##
###########
include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  hal_motor_control_interfaces
  common_utils)

add_library(${PROJECT_NAME} SHARED src/hal_pose_manager.cpp)
  ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(${PROJECT_NAME}_node
  src/hal_pose_manager_node.cpp
  $<TARGET_OBJECTS:${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/test_hal_pose_manager.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test ${dependencies})
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_dependencies(${dependencies})

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
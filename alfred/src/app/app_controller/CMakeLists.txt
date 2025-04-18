cmake_minimum_required(VERSION 3.8)
project(app_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
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
  geometry_msgs
  nav_msgs
  common_utils)

add_library(${PROJECT_NAME} OBJECT src/app_controller.cpp)
  ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(${PROJECT_NAME}_node
  src/app_controller_node.cpp
  $<TARGET_OBJECTS:${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/test_app_controller.cpp)
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

ament_package()
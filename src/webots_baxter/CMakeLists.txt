cmake_minimum_required(VERSION 3.8)
project(webots_baxter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(my_opencv_demo REQUIRED)
find_package(pymoveit2 REQUIRED)


#Add Source files
add_library(${PROJECT_NAME} SHARED
src/first_node.cpp
)


#Install Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  rclcpp_action
  std_msgs
  "geometry_msgs"
  webots_ros2_driver
  "pluginlib"
)

#Install Executables
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "webots_baxter::FirstNode"
  EXECUTABLE first_node
)

# Install Launch files
install(DIRECTORY launch config worlds proto resource DESTINATION share/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(my_opencv_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(opencv_interfaces REQUIRED)



add_library(${PROJECT_NAME} SHARED
  src/cpp_opencv_demo.cpp
  )

ament_target_dependencies(${PROJECT_NAME}
  rclcpp 
  rclcpp_components 
  std_msgs 
  sensor_msgs 
  cv_bridge 
  image_transport 
  OpenCV
  )

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "my_opencv_demo::CppOpenCvDemo"
  EXECUTABLE cpp_opencv_demo
)


#Install Python Modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
scripts/block_detection.py
scripts/block_detection_service.py
DESTINATION lib/${PROJECT_NAME})



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

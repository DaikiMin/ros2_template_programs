cmake_minimum_required(VERSION 3.8)
project(ros2_template_programs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_custom_msg REQUIRED)

#--- for C++
# Include Cpp "include" directory
include_directories(include)
# Create Cpp executable
add_executable(topic_publisher_template src/topic_publisher_template.cpp)
ament_target_dependencies(topic_publisher_template rclcpp std_msgs)

add_executable(topic_subscriber_template src/topic_subscriber_template.cpp)
ament_target_dependencies(topic_subscriber_template rclcpp std_msgs)

# Install Cpp executables
install(TARGETS
  topic_publisher_template
  topic_subscriber_template
  DESTINATION lib/${PROJECT_NAME}
)
#--- end C++

#-- for Python
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/topic_publisher_template.py
  scripts/topic_subscriber_template.py
  scripts/service_client_template.py
  DESTINATION lib/${PROJECT_NAME}
)
#--- end Python

# For code check
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
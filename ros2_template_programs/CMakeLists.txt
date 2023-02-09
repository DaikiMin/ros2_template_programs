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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

#--- for C++
# Include Cpp "include" directory
include_directories(include)
# Create Cpp executable
add_executable(topic_publisher_template src/topic_publisher_template.cpp)
ament_target_dependencies(topic_publisher_template rclcpp std_msgs)

add_executable(topic_subscriber_template src/topic_subscriber_template.cpp)
ament_target_dependencies(topic_subscriber_template rclcpp std_msgs)

add_executable(service_client_template src/service_client_template.cpp)
ament_target_dependencies(service_client_template rclcpp ros2_custom_msg)

add_executable(service_client_from_callback_template src/service_client_from_callback_template.cpp)
ament_target_dependencies(service_client_from_callback_template rclcpp ros2_custom_msg)

add_executable(service_server_template src/service_server_template.cpp)
ament_target_dependencies(service_server_template rclcpp ros2_custom_msg)

# Install Cpp executables
install(TARGETS
  topic_publisher_template
  topic_subscriber_template
  service_client_template
  service_client_from_callback_template
  service_server_template
  DESTINATION lib/${PROJECT_NAME}
)

# For Action
add_library(action_server_template SHARED
  src/action_server_template.cpp)
target_include_directories(action_server_template PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server_template
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_server_template
  "ros2_custom_msg"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server_template PLUGIN "ros2_template_programs::ActionServer" EXECUTABLE timer_action_server)
install(TARGETS
  action_server_template
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(action_client_template SHARED
  src/action_client_template.cpp)
target_include_directories(action_client_template PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client_template
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client_template
  "ros2_custom_msg"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client_template PLUGIN "ros2_template_programs::ActionClient" EXECUTABLE timer_action_client)
install(TARGETS
  action_client_template
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

#--- end C++

#-- for Python
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/topic_publisher_template.py
  scripts/topic_subscriber_template.py
  scripts/service_client_template.py
  scripts/service_client_from_callback_template.py
  scripts/service_server_template.py
  scripts//action_client_template.py
  scripts/action_server_template.py
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
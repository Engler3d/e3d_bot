cmake_minimum_required(VERSION 3.8)
project(e3d_execute)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(e3d_bot_msgs REQUIRED)
find_package(arduinobot_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add library (e3d_bot_task_server)
add_library(e3d_bot_task_server SHARED src/e3d_bot_task_server.cpp)
target_include_directories(e3d_bot_task_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(e3d_bot_task_server
  PRIVATE "E3D_BOT_TASK_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(e3d_bot_task_server
  "arduinobot_msgs"
  "e3d_bot_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface")
rclcpp_components_register_node(e3d_bot_task_server 
  PLUGIN "e3d_execute::E3DTaskServer" 
  EXECUTABLE e3d_task_server_node
)

# Add executable (path_follower)
add_executable(path_follower src/path_follower.cpp)
ament_target_dependencies(path_follower
  "rclcpp"
  "moveit_ros_planning_interface"
  "geometry_msgs"
)

ament_python_install_package(${PROJECT_NAME})

# Comment out if you don't have a launch directory
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install targets
install(TARGETS 
  e3d_bot_task_server
  path_follower
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(agv_proto)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  ros_odrive_msg
  jsoncpp
  angles
  geometry_msgs
  nav2_msgs
  rclcpp_action
)

include_directories(
  include
  ${jsoncpp_INCLUDE_DIRS}
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  agv_diff_control
  SHARED
  src/robot_hardware_interface_agv.cpp src/odrive_utils.cpp src/odrive_endpoint.cpp
)

target_compile_features(agv_diff_control PUBLIC cxx_std_17)
target_include_directories(agv_diff_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/agv_proto>
)

ament_target_dependencies(agv_diff_control PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(agv_diff_control
  PUBLIC
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${ros_odrive_msg_LIBRARIES}
  ${jsoncpp_LIBRARIES}
  usb-1.0 
  -L${jsoncpp_LIBRARY_DIRS}
  -ljsoncpp
  ${catkin_LIBRARIES}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(agv_diff_control PRIVATE "AGV_DIFF_CONTROL_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface agv_diff_control.xml)



add_executable(my_nav2_goal_client src/my_nav2_goal_client.cpp)
ament_target_dependencies(my_nav2_goal_client rclcpp nav2_msgs rclcpp_action std_msgs)

target_link_libraries(my_nav2_goal_client
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${Boost_LIBRARIES} 
)


# INSTALL
install(
  DIRECTORY include/agv_proto
  DESTINATION include/agv_proto
)

install(
  DIRECTORY launch rviz urdf config maps
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS agv_diff_control
  EXPORT export_agv_diff_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  my_nav2_goal_client
  DESTINATION lib/${PROJECT_NAME}
)


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

ament_export_targets(export_agv_diff_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

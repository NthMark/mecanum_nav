cmake_minimum_required(VERSION 3.8)
project(mecanum)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
install(
  DIRECTORY config description launch urdf meshes world
  DESTINATION share/${PROJECT_NAME}
)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(control src/control.cpp)
ament_target_dependencies(control rclcpp geometry_msgs)
add_executable(control_lifecycle
  src/control_lifecycle.cpp)
ament_target_dependencies(control_lifecycle
  lifecycle_msgs
  rclcpp_lifecycle
  geometry_msgs
  rclcpp
  tf2
  tf2_ros 
  nav_msgs
  nav2_msgs
)
add_executable(followpath_lifecycle
  src/followpath_lifecycle.cpp)
ament_target_dependencies(followpath_lifecycle
  lifecycle_msgs
  rclcpp_lifecycle
  geometry_msgs
  rclcpp
  tf2
  tf2_ros 
  nav_msgs
)
add_executable(fixed_frame
  src/fixed_frame.cpp
)
ament_target_dependencies(fixed_frame
  geometry_msgs
  rclcpp
  tf2
  tf2_ros 
)
add_executable(view_frames
src/view_frames.cpp
)
ament_target_dependencies(view_frames
  geometry_msgs
  rclcpp
  tf2
  tf2_ros 
)
include_directories(include)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# add_library(navigator SHARED
#   src/navigator.cpp)
#   rclcpp_components_register_node(mecanum PLUGIN "navigator::BasicNavigator" EXECUTABLE navigator)
# target_compile_definitions(navigator
#   PRIVATE "NAVIGATOR_CPP_BUILDING_DLL")
# ament_target_dependencies(navigator
#   geometry_msgs
#   rclcpp
#   lifecycle_msgs
#   builtin_interfaces
#   rclcpp_action
#   nav2_msgs
#   rclcpp_components)

# install(TARGETS
#   action_tutorials
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)
install(TARGETS
  control
  control_lifecycle
  followpath_lifecycle
  fixed_frame
  view_frames
  # navigator
  DESTINATION lib/${PROJECT_NAME})
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()

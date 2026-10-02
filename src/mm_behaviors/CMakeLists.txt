cmake_minimum_required(VERSION 3.8)
project(mm_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_library(ESPEAK_LIB espeak)


install(DIRECTORY
  xml_tree launch config
  DESTINATION share/${PROJECT_NAME}
)


set( NAV2_BT_SOURCES
    src/mm_nav_actions.cpp
)
set( MOVEIT_BT_SOURCES
    src/mm_moveit_bt_nodes.cpp
)
set( OTHER_BT_SOURCES
    src/actions_nodes.cpp
    src/condition_nodes.cpp
)
set(TARGET_DEPENDS
    rclcpp rclcpp_action std_msgs geometry_msgs behaviortree_cpp 
    moveit_ros_planning_interface moveit_core moveit_ros_planning
    nav2_msgs behaviortree_cpp tf2 tf2_ros tf2_geometry_msgs

)

include_directories(include)

add_executable(wstta_bt_node src/mm_bt_node.cpp ${NAV2_BT_SOURCES} ${MOVEIT_BT_SOURCES} ${OTHER_BT_SOURCES})
ament_target_dependencies(wstta_bt_node ${TARGET_DEPENDS})
target_link_libraries(wstta_bt_node
  ${ESPEAK_LIB} 
)
target_include_directories(wstta_bt_node PUBLIC
  ${OMPL_INCLUDE_DIRS}
)


install(TARGETS
    wstta_bt_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)


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

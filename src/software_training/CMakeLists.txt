cmake_minimum_required(VERSION 3.8)
project(software_training)

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

# -------------------------------------------------------------------
# find packages 
# -------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -------------------------------------------------------------------
# include, generate interfaces, and setup plugins 
# -------------------------------------------------------------------
include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/Reset.srv"
  "msg/Distance.msg"
  "action/Waypoint.action"
	DEPENDENCIES std_msgs geometry_msgs builtin_interfaces # not sure if i need geometry?
	)
ament_export_dependencies(rosidl_default_runtime)

#create resource which references the libraries in the binary bin
set(node_plugins "")

# -------------------------------------------------------------------
# P1
# -------------------------------------------------------------------
add_library(p1 SHARED
            src/p1_clear.cpp)

target_compile_definitions(p1 PRIVATE "SOFTWARE_TRAINING_BUILDING_DLL")

ament_target_dependencies(p1
  "rclcpp"
  "rclcpp_components"
  "turtlesim"
  "geometry_msgs"
  "std_msgs")   
rclcpp_components_register_nodes(p1 "composition::p1_clear")
set(node_plugins "${node_plugins}composition::p1_clear;$<TARGET_FILE:p1>\n")

# -------------------------------------------------------------------
# P2
# -------------------------------------------------------------------
add_library(p2 SHARED
            src/p2_circle.cpp)

target_compile_definitions(p2 PRIVATE "SOFTWARE_TRAINING_BUILDING_DLL")

ament_target_dependencies(p2
  "rclcpp"
  "rclcpp_components"
  "turtlesim"
  "geometry_msgs"
  "std_msgs")   
rclcpp_components_register_nodes(p2 "composition::p2_circle")
set(node_plugins "${node_plugins}composition::p2_circle;$<TARGET_FILE:p2>\n")

# -------------------------------------------------------------------
# P3
# -------------------------------------------------------------------
add_library(p3 SHARED
            src/p3_spawn.cpp)

target_compile_definitions(p3 PRIVATE "SOFTWARE_TRAINING_BUILDING_DLL")

ament_target_dependencies(p3
  "rclcpp"
  "rclcpp_components"
  "turtlesim"
  "std_msgs")   
rclcpp_components_register_nodes(p3 "composition::p3_spawn")
set(node_plugins "${node_plugins}composition::p3_spawn;$<TARGET_FILE:p3>\n")

# -------------------------------------------------------------------
# P4
# -------------------------------------------------------------------
add_library(p4 SHARED
            src/p4_reset.cpp)

target_compile_definitions(p4 PRIVATE "SOFTWARE_TRAINING_BUILDING_DLL")

ament_target_dependencies(p4
  "rclcpp"
  "rclcpp_components"
  "turtlesim"
  "std_msgs")   

rosidl_target_interfaces(p4 ${PROJECT_NAME} "rosidl_typesupport_cpp") # need for custom srv

rclcpp_components_register_nodes(p4 "composition::p4_reset")
set(node_plugins "${node_plugins}composition::p4_reset;$<TARGET_FILE:p4\n>")

# -------------------------------------------------------------------
# P5
# -------------------------------------------------------------------
add_library(p5 SHARED
            src/p5_distance.cpp)

target_compile_definitions(p5 PRIVATE "SOFTWARE_TRAINING_BUILDING_DLL")

ament_target_dependencies(p5
  "rclcpp"
  "rclcpp_components"
  "turtlesim"
  "std_msgs")   

rosidl_target_interfaces(p5 ${PROJECT_NAME} "rosidl_typesupport_cpp") # need for custom srv

rclcpp_components_register_nodes(p5 "composition::p5_distance")
set(node_plugins "${node_plugins}composition::p5_distance;$<TARGET_FILE:p5\n>")

# -------------------------------------------------------------------
# P6
# -------------------------------------------------------------------
add_library(p6 SHARED
            src/p6_waypoint.cpp)

target_compile_definitions(p6 PRIVATE "SOFTWARE_TRAINING_BUILDING_DLL")

ament_target_dependencies(p6
  "rclcpp"
  "rclcpp_components"
  "rclcpp_action"
  "turtlesim"
  "geometry_msgs"
  "std_msgs")   

rosidl_target_interfaces(p6 ${PROJECT_NAME} "rosidl_typesupport_cpp") # need for custom srv

rclcpp_components_register_nodes(p6 "composition::p6_waypoint")
set(node_plugins "${node_plugins}composition::p6_waypoint;$<TARGET_FILE:p6\n>")


# rclcpp_components_register_nodes(p6 PLUGIN "composition::p6_waypoint"
#                                     EXECUTABLE p6_waypoint_run)

# -------------------------------------------------------------------
# Install binary locations
# -------------------------------------------------------------------

install(TARGETS
  p1	
  p2
  p3
  p4
  p5
  p6 

	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin)

# -------------------------------------------------------------------
# Launch file
# -------------------------------------------------------------------

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
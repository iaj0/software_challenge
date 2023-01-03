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
# find dependencies
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
	"msg/Turtle.msg"
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
  "std_msgs")   
rclcpp_components_register_nodes(p1 "composition::p1_clear")
set(node_plugins "${node_plugins}composition::p1_clear;$<TARGET_FILE:p1>\n")


# -------------------------------------------------------------------
# Install binary locations
# -------------------------------------------------------------------

install(TARGETS
  p1	

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
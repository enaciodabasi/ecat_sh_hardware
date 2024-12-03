cmake_minimum_required(VERSION 3.3.0)
project(ecat_sh_hardware VERSION 0.1 LANGUAGES C CXX)

set(CXX_STANDARD 23)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /usr /usr/local /opt/ros/humble/include/)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra  -Werror=return-type)
endif()
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_compile_options(-std=c++23)

find_package(Threads REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED COMPONENTS LinearMath)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ipc_handlers REQUIRED)
find_package(timespec_chrono_conversions REQUIRED)
find_package(cppzmq REQUIRED)

message(ipc_handlers_INCLUDE_DIRS="${IPC_HANDLERS_INCLUDE_DIRS}")


add_executable(
	diff_drive_hardware
	src/ecat_hardware.cpp
  src/utils.cpp
)
target_include_directories(diff_drive_hardware PUBLIC include/ ${ipc_handlers_INCLUDE_DIRS} ${timespec_chrono_conversions_INCLUDE_DIRS}/timespec_chrono_conversions)
target_link_libraries(diff_drive_hardware ethercat rt Threads::Threads ${ipc_handlers_LIBRARIES})

add_executable(
  diff_drive_controller
  src/controller.cpp
  src/kinematics.cpp
  src/ros_communication.cpp
)
target_link_libraries(diff_drive_controller rt Threads::Threads ${ipc_handlers_LIBRARIES} ${rclcpp_LIBRARIES} ${rcl_interfaces_LIBRARIES} ${rosidl_typesupport_cpp_LIBRARIES} ${rosidl_default_runtime_LIBRARIES} ${tf2_LIBRARIES} ${geometry_msgs} ${nav_msgs_LIBRARIES} ${tf2_msgs})
target_include_directories(
  diff_drive_controller 
  PUBLIC 
  include/ 
  ${ipc_handlers_INCLUDE_DIRS} 
  ${timespec_chrono_conversions_INCLUDE_DIRS}/timespec_chrono_conversions 
  ${rclcpp_INCLUDE_DIRS} 
  ${rosidl_typesupport_cpp_INCLUDE_DIRS} 
  ${tf2_INCLUDE_DIRS} 
  ${geometry_msgs_INCLUDE_DIRS} 
  ${nav_msgs_INCLUDE_DIRS} 
  ${tf2_msgs_INCLUDE_DIRS}
)

install(
	TARGETS
	diff_drive_hardware diff_drive_controller
	DESTINATION
	${CMAKE_CURRENT_BINARY_DIR}/build
)


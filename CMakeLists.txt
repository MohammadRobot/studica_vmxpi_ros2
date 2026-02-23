cmake_minimum_required(VERSION 3.16)
project(vmxpi_ros2 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Find VMXPi library and headers (CONDITIONAL)
find_package(studica_drivers QUIET)

if(studica_drivers_FOUND)
  get_target_property(STUDICA_DRIVERS_TARGET_TYPE studica_drivers::studica_drivers TYPE)
  if(STUDICA_DRIVERS_TARGET_TYPE STREQUAL "INTERFACE_LIBRARY")
    message(WARNING "studica_drivers built without VMXPi support. Skipping hardware interface.")
    set(VMXPI_FOUND FALSE)
    set(VMXPI_LIBRARIES "")
    set(VMXPI_INCLUDE_DIRS "")
  else()
    message(STATUS "studica_drivers package FOUND: Using workspace driver library.")
    set(VMXPI_FOUND TRUE)
    set(VMXPI_LIBRARIES studica_drivers::studica_drivers)
    set(VMXPI_INCLUDE_DIRS "")
  endif()
else()
  find_library(VMXPI_LIBRARY
    NAMES studica_drivers
    HINTS /usr/local/lib/studica_drivers
          /usr/local/lib
    )

  find_path(VMXPI_INCLUDE_DIR
    NAMES titan.h
    HINTS /usr/local/include/studica_drivers
          /usr/local/include
    )

  if(VMXPI_LIBRARY AND VMXPI_INCLUDE_DIR)
    message(STATUS "VMXPi library and headers FOUND: Using system driver library.")
    set(VMXPI_FOUND TRUE)
    set(VMXPI_LIBRARIES ${VMXPI_LIBRARY})
    set(VMXPI_INCLUDE_DIRS ${VMXPI_INCLUDE_DIR})
  else()
    message(WARNING "VMXpi library and headers NOT FOUND: Building without hardware interface support.")
    set(VMXPI_FOUND FALSE)
    set(VMXPI_LIBRARIES "")
    set(VMXPI_INCLUDE_DIRS "")
  endif()
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  vmxpi_ros2
  SHARED
  hardware/diffbot_system.cpp
)

if(VMXPI_FOUND)
  include_directories(
    include
    ${VMXPI_INCLUDE_DIRS}
    /usr/local/include/vmxpi
  )

  target_sources(vmxpi_ros2
    PRIVATE
    hardware/titan_system.cpp
  )

  target_link_libraries(vmxpi_ros2
    PUBLIC
    ${VMXPI_LIBRARIES}
  )
else()
  include_directories(include)
  message(WARNING "Skipping compilation of hardware interface code.")
endif()

target_compile_features(vmxpi_ros2 PUBLIC cxx_std_17)
target_include_directories(vmxpi_ros2 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/vmxpi_ros2>
)
ament_target_dependencies(
  vmxpi_ros2 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface vmxpi_ros2.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/vmxpi_ros2
)
install(
  DIRECTORY description
  DESTINATION share/vmxpi_ros2
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/vmxpi_ros2
)

add_executable(robot_patrol_node src/patrol.cpp)
ament_target_dependencies(robot_patrol_node rclcpp sensor_msgs geometry_msgs)

add_executable(nav2_topic_bridge_node src/nav2_topic_bridge.cpp)
ament_target_dependencies(nav2_topic_bridge_node rclcpp geometry_msgs nav_msgs)

install(TARGETS
   robot_patrol_node
   nav2_topic_bridge_node
   DESTINATION lib/${PROJECT_NAME}
 )

install(TARGETS vmxpi_ros2
  EXPORT export_vmxpi_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_vmxpi_ros2 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

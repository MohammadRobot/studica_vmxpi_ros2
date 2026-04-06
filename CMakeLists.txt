cmake_minimum_required(VERSION 3.16)
project(studica_vmxpi_ros2 LANGUAGES CXX)
include(CTest)

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
  find_library(
    VMXPI_LIBRARY
    NAMES studica_drivers
    HINTS
      /usr/local/lib/studica_drivers
      /usr/local/lib
  )

  find_path(
    VMXPI_INCLUDE_DIR
    NAMES titan.h
    HINTS
      /usr/local/include/studica_drivers
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
#
# Keep ROS builds isolated from an active Conda environment. Conda can expose
# fmt/spdlog configs and libstdc++ that are ABI-incompatible with system ROS.
set(_conda_prefixes "")
if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  list(APPEND _conda_prefixes "$ENV{CONDA_PREFIX}")
endif()
if(DEFINED ENV{CONDA_EXE} AND NOT "$ENV{CONDA_EXE}" STREQUAL "")
  get_filename_component(_conda_bin_dir "$ENV{CONDA_EXE}" DIRECTORY)
  get_filename_component(_conda_root "${_conda_bin_dir}/.." ABSOLUTE)
  list(APPEND _conda_prefixes "${_conda_root}")
endif()
list(REMOVE_DUPLICATES _conda_prefixes)
foreach(_conda_prefix IN LISTS _conda_prefixes)
  if(EXISTS "${_conda_prefix}")
    list(APPEND CMAKE_IGNORE_PREFIX_PATH "${_conda_prefix}")
    list(APPEND CMAKE_IGNORE_PATH
      "${_conda_prefix}"
      "${_conda_prefix}/bin"
      "${_conda_prefix}/lib"
      "${_conda_prefix}/lib64"
      "${_conda_prefix}/lib/cmake"
      "${_conda_prefix}/share"
      "${_conda_prefix}/include"
    )

    foreach(_pkg_dir_var IN ITEMS fmt_DIR spdlog_DIR)
      if(DEFINED ${_pkg_dir_var})
        string(FIND "${${_pkg_dir_var}}" "${_conda_prefix}" _conda_hit)
        if(NOT _conda_hit EQUAL -1)
          message(STATUS "Clearing cached ${_pkg_dir_var} from Conda path: ${${_pkg_dir_var}}")
          unset(${_pkg_dir_var} CACHE)
          unset(${_pkg_dir_var})
        endif()
      endif()
    endforeach()
  endif()
endforeach()
unset(_conda_hit)
unset(_pkg_dir_var)
unset(_conda_root)
unset(_conda_bin_dir)
unset(_conda_prefix)
unset(_conda_prefixes)

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
# Avoid Conda Python injecting non-system runtime paths into ROS binaries.
set(Python3_FIND_VIRTUALENV STANDARD)
set(Python3_FIND_STRATEGY LOCATION)
if(EXISTS "/usr/bin/python3")
  set(Python3_ROOT_DIR "/usr" CACHE PATH "System Python root" FORCE)
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "System Python3 executable" FORCE)
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  studica_vmxpi_ros2
  SHARED
  hardware/sim_system.cpp
)

if(VMXPI_FOUND)
  target_sources(studica_vmxpi_ros2
    PRIVATE
    hardware/vmx_system.cpp
  )

  target_include_directories(studica_vmxpi_ros2 PRIVATE
    ${VMXPI_INCLUDE_DIRS}
    /usr/local/include/vmxpi
  )

  target_link_libraries(studica_vmxpi_ros2
    PUBLIC
    ${VMXPI_LIBRARIES}
  )
else()
  message(WARNING "Skipping compilation of hardware interface code.")
endif()

target_compile_features(studica_vmxpi_ros2 PUBLIC cxx_std_17)
target_include_directories(studica_vmxpi_ros2 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/studica_vmxpi_ros2>
)
ament_target_dependencies(
  studica_vmxpi_ros2 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface studica_vmxpi_ros2.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/studica_vmxpi_ros2
)
install(
  DIRECTORY description
  DESTINATION share/studica_vmxpi_ros2
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/studica_vmxpi_ros2
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  DIRECTORY docs
  DESTINATION share/studica_vmxpi_ros2
)
install(PROGRAMS
  scripts/motor_smoke_test.sh
  scripts/create_profile.sh
  scripts/install_git_hooks.sh
  scripts/validate_profiles.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(robot_patrol_node src/patrol.cpp)
ament_target_dependencies(robot_patrol_node rclcpp sensor_msgs geometry_msgs)
target_compile_features(robot_patrol_node PUBLIC cxx_std_17)

add_executable(topic_adapter_node src/topic_adapter_node.cpp)
ament_target_dependencies(topic_adapter_node rclcpp geometry_msgs nav_msgs sensor_msgs tf2_msgs)
target_compile_features(topic_adapter_node PUBLIC cxx_std_17)

install(TARGETS
  robot_patrol_node
  topic_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS studica_vmxpi_ros2
  EXPORT export_studica_vmxpi_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  option(STUDICA_STRICT_LINT "Enable full strict lint checks" OFF)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  if(NOT STUDICA_STRICT_LINT)
    # Legacy codebase: enable lint infrastructure first, then tighten checks incrementally.
    set(AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_copyright
      ament_cmake_cpplint
      ament_cmake_flake8
      ament_cmake_uncrustify
    )
  endif()
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_test REQUIRED)
  ament_add_test(
    validate_robot_profiles
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_profiles.py
      --profiles-dir
      ${CMAKE_CURRENT_SOURCE_DIR}/bringup/config/profiles
  )
endif()

## EXPORTS
ament_export_targets(export_studica_vmxpi_ros2 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

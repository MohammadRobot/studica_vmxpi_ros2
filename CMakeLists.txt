cmake_minimum_required(VERSION 3.16)
project(studica_vmxpi_ros2 LANGUAGES CXX)
include(CTest)
option(
  STUDICA_PRODUCTION_INSTALL
  "Install only the VMXPi production runtime assets"
  OFF
)

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

if(STUDICA_PRODUCTION_INSTALL AND NOT VMXPI_FOUND)
  message(FATAL_ERROR
    "STUDICA_PRODUCTION_INSTALL requires the VMXPi hardware interface. "
    "Build it in the qualified ARM64 hardware workspace."
  )
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
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
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
if(STUDICA_PRODUCTION_INSTALL)
  install(
    DIRECTORY
      description/meshes
      description/ros2_control
      description/urdf
    DESTINATION share/studica_vmxpi_ros2/description
  )
  install(
    DIRECTORY description/robot/urdf
    DESTINATION share/studica_vmxpi_ros2/description/robot
  )
  install(
    FILES
      bringup/launch/_camera_hw.launch.py
      bringup/launch/_classroom_launch.py
      bringup/launch/_launch_gz.py
      bringup/launch/_launch_helpers.py
      bringup/launch/_launch_sensors.py
      bringup/launch/_lidar_hw.launch.py
      bringup/launch/_robot_runtime.launch.py
      bringup/launch/bringup.launch.py
      bringup/launch/profile_validation.py
      bringup/launch/robot.launch.py
    DESTINATION share/studica_vmxpi_ros2/launch
  )
  install(
    FILES
      bringup/config/dualshock4_teleop.yaml
      bringup/config/hardware_imu_odometry.yaml
    DESTINATION share/studica_vmxpi_ros2/config
  )
  install(
    DIRECTORY bringup/config/profiles/stack_4wd
    DESTINATION share/studica_vmxpi_ros2/config/profiles
  )
  install(
    FILES deployment/vmxpi-production-install-v1.json.in
    DESTINATION share/studica_vmxpi_ros2/deployment
    RENAME vmxpi-production-install-v1.json
  )
  install(PROGRAMS
    scripts/audit_vmxpi_runtime.py
    scripts/point_cloud_filter.py
    DESTINATION lib/${PROJECT_NAME}
  )
else()
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
  install(
    DIRECTORY foxglove
    DESTINATION share/studica_vmxpi_ros2
  )
  install(
    DIRECTORY maps
    DESTINATION share/studica_vmxpi_ros2
  )
  install(
    DIRECTORY dependencies deployment examples
    DESTINATION share/studica_vmxpi_ros2
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
  )
  install(PROGRAMS
    scripts/audit_vmxpi_runtime.py
    scripts/build_release_bundle.py
    scripts/configure_cyclonedds.py
    scripts/create_profile.sh
    scripts/depth_to_pointcloud.py
    scripts/install_git_hooks.sh
    scripts/point_cloud_filter.py
    scripts/run_waypoint_route.py
    scripts/check_project.sh
    scripts/validate_production_manifest.py
    scripts/validate_profiles.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

add_executable(topic_adapter_node src/topic_adapter_node.cpp)
ament_target_dependencies(
  topic_adapter_node builtin_interfaces rclcpp geometry_msgs nav_msgs sensor_msgs tf2_msgs)
target_include_directories(topic_adapter_node PRIVATE hardware/include)
target_compile_features(topic_adapter_node PUBLIC cxx_std_17)

add_executable(safety_supervisor_node src/safety_supervisor_node.cpp)
ament_target_dependencies(
  safety_supervisor_node control_msgs rcl_interfaces rclcpp geometry_msgs sensor_msgs std_msgs
  std_srvs)
target_include_directories(safety_supervisor_node PRIVATE hardware/include)
target_compile_features(safety_supervisor_node PUBLIC cxx_std_17)

if(VMXPI_FOUND)
  add_executable(safety_input_check tools/safety_input_check.cpp)
  target_include_directories(safety_input_check PRIVATE
    ${VMXPI_INCLUDE_DIRS}
    /usr/local/include/vmxpi
  )
  target_link_libraries(safety_input_check PRIVATE ${VMXPI_LIBRARIES})
  target_compile_features(safety_input_check PUBLIC cxx_std_17)
  install(TARGETS safety_input_check DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS
  safety_supervisor_node
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
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu_conventions test/test_imu_conventions.cpp)
  target_include_directories(test_imu_conventions PRIVATE hardware/include)
  target_compile_features(test_imu_conventions PUBLIC cxx_std_17)
  ament_add_gtest(test_velocity_pid_safety test/test_velocity_pid_safety.cpp)
  target_include_directories(test_velocity_pid_safety PRIVATE hardware/include)
  ament_add_gtest(test_topic_adapter test/test_topic_adapter.cpp)
  target_include_directories(test_topic_adapter PRIVATE hardware/include)
  ament_target_dependencies(
    test_topic_adapter builtin_interfaces geometry_msgs nav_msgs sensor_msgs)
  ament_add_gtest(test_safety_supervisor test/test_safety_supervisor.cpp)
  target_include_directories(test_safety_supervisor PRIVATE hardware/include)
  target_compile_features(test_safety_supervisor PUBLIC cxx_std_17)
  ament_add_gtest(test_local_enable_gate test/test_local_enable_gate.cpp)
  target_include_directories(test_local_enable_gate PRIVATE hardware/include)
  target_compile_features(test_local_enable_gate PUBLIC cxx_std_17)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.flake8")
  if(NOT STUDICA_STRICT_LINT)
    # Legacy codebase: enable lint infrastructure first, then tighten checks incrementally.
    set(AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_copyright
      ament_cmake_cpplint
      ament_cmake_uncrustify
    )
  endif()
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_test REQUIRED)
  ament_add_test(
    validate_depth_to_pointcloud
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_depth_to_pointcloud.py
  )
  ament_add_test(
    validate_point_cloud_filter
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_point_cloud_filter.py
  )
  ament_add_test(
    validate_nav2_perception
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_nav2_perception.py
  )
  ament_add_test(
    validate_robot_profiles
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_profiles.py
      --profiles-dir
      ${CMAKE_CURRENT_SOURCE_DIR}/bringup/config/profiles
  )
  ament_add_test(
    validate_profile_safety_gate
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_profile_safety_gate.py
  )
  ament_add_test(
    validate_beginner_launches
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_beginner_launches.py
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_test(
    validate_classroom_source
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_project.sh
  )
  ament_add_test(
    validate_setup_contract
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_setup_contract.py
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_test(
    validate_vmxpi_runtime_audit
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vmxpi_runtime_audit.py
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_test(
    validate_production_package_manifest
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_production_manifest.py
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_test(
    validate_release_bundle
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_release_bundle.py
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_test(
    validate_arm64_builder
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_arm64_builder.py
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_test(
    validate_release_artifacts
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_release_artifacts.py
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_test(
    validate_python_examples
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/examples/python/tests/test_python_examples.py
  )
  set(_mock_runtime_arguments --mode mock)
  if(STUDICA_PRODUCTION_INSTALL)
    # Production artifacts intentionally contain only their qualified hardware
    # profile. Exercise that installed profile instead of the teaching default.
    list(APPEND _mock_runtime_arguments --robot-profile stack_4wd)
  endif()
  ament_add_test(
    validate_mock_runtime
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 90
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_runtime_contract.py
      ${_mock_runtime_arguments}
  )
  unset(_mock_runtime_arguments)
  ament_add_test(
    validate_hardware_supervisor_runtime
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 30
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_hardware_supervisor_runtime.py
  )
  ament_add_test(
    validate_joystick_supervisor_runtime
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 40
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_joystick_supervisor_runtime.py
  )
  ament_add_test(
    validate_headless_gz_runtime
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 150
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_runtime_contract.py
      --mode gz
      --allow-skip
  )
  foreach(classroom_mode IN ITEMS mapping navigation)
    ament_add_test(
      validate_headless_${classroom_mode}
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 180
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_runtime_contract.py
        --mode ${classroom_mode}
        --allow-skip
    )
  endforeach()
endif()

## EXPORTS
ament_export_targets(export_studica_vmxpi_ros2 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

cmake_minimum_required(VERSION 3.16)
project(volcanibot_diff_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Find libserial
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

## COMPILE
add_library(
  volcanibot_diff_controller
  SHARED
  src/diffdrive_system.cpp
)
target_compile_features(volcanibot_diff_controller PUBLIC cxx_std_17)
target_include_directories(volcanibot_diff_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/volcanibot_diff_controller>
)
target_include_directories(volcanibot_diff_controller PUBLIC ${LIBSERIAL_INCLUDE_DIRS})
ament_target_dependencies(
  volcanibot_diff_controller PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(volcanibot_diff_controller PUBLIC ${LIBSERIAL_LIBRARIES})
target_compile_options(volcanibot_diff_controller PUBLIC ${LIBSERIAL_CFLAGS_OTHER})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFFDRIVE_ROS2_CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface volcanibot_diff_controller.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/volcanibot_diff_controller
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/volcanibot_diff_controller/description
)
install(
  DIRECTORY launch bringup/config    #bringup/launch
  DESTINATION share/volcanibot_diff_controller
)
install(
  PROGRAMS scripts/cmd_vel_converter.py scripts/yolo_joy_toggle.py
  DESTINATION lib/volcanibot_diff_controller
)
install(TARGETS volcanibot_diff_controller
  EXPORT export_volcanibot_diff_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

## EXPORTS
ament_export_targets(export_volcanibot_diff_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

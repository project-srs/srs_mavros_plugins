cmake_minimum_required(VERSION 3.8)
project(srs_mavros_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/led_control.cpp
  src/velocity_position.cpp
  src/play_tune_v1.cpp
  src/rc_joy.cpp
  src/reboot.cpp
  src/setup.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  # must for plugin build
  mavros
  mavros_msgs
  pluginlib
  diagnostic_msgs
  tf2_ros
  # use in plugins
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(mavros mavros_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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

cmake_minimum_required(VERSION 3.8)
project(biped_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============== find dependencies ================
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(biped_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(biped_fsm REQUIRED)
find_package(parameter_traits REQUIRED)
find_package(generate_parameter_library REQUIRED)


# ============== generate parameter library ================
generate_parameter_library(
  blind_walk_controller_parameters
  src/blind_walk_controller.yaml
)

# =============== build controller library ================
add_library(blind_walk_controller
  SHARED
  src/blind_walk_controller.cpp
)
# link the parameter library to the controller library
target_link_libraries(blind_walk_controller
  blind_walk_controller_parameters
)
#include directories for the controller library
target_include_directories(blind_walk_controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# link dependencies for the controller library
ament_target_dependencies(blind_walk_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  biped_msgs
  geometry_msgs
  biped_fsm
  parameter_traits
  generate_parameter_library
)



# =============== export plugin description ================
# export the plugin description file for the controller
pluginlib_export_plugin_description_file(controller_interface biped_control.xml)


# =============== install library ================
install(TARGETS blind_walk_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# =============== install headers ================
install(
  DIRECTORY
  include/
  DESTINATION include
)

# =============== install plugin description file ================
install(FILES
  biped_control.xml
  DESTINATION share/${PROJECT_NAME}
)

# =============== install launch/config files ================
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
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

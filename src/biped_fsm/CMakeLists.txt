cmake_minimum_required(VERSION 3.8)
project(biped_fsm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(yasmin REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(yasmin_factory REQUIRED)
find_package(pluginlib REQUIRED)


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

# ---------- Python executables -----------
# Install Python modules
ament_python_install_package(${PROJECT_NAME})


# Install Python executables (ROS nodes)
install(PROGRAMS
  biped_fsm/state_manager_node.py
  biped_fsm/test_shm_state_manager.py
  DESTINATION lib/${PROJECT_NAME}
)
# -------------------------------------



# Build the shared library for the plugins
add_library(biped_fsm_plugins SHARED src/demo_states_plugin.cpp)
target_include_directories(biped_fsm_plugins PUBLIC include)
ament_target_dependencies(biped_fsm_plugins 
  rclcpp 
  yasmin 
  pluginlib
)

# 3. Tell ROS 2 where the plugin XML file is
pluginlib_export_plugin_description_file(yasmin plugins.xml)

# 4. Install the plugin library
install(TARGETS biped_fsm_plugins
  DESTINATION lib/${PROJECT_NAME}
)

# ---------- c++ executable -----------
add_executable(simple_fsm_demo src/simple_fsm_demo.cpp)
ament_target_dependencies(simple_fsm_demo yasmin rclcpp yasmin_viewer yasmin_factory)


install(TARGETS
  simple_fsm_demo
  DESTINATION lib/${PROJECT_NAME}
)
# -------------------------------------

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

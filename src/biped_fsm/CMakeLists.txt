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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(iox2_msgs REQUIRED)
find_package(iceoryx2-cxx REQUIRED)

install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

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
add_library(biped_fsm_plugins SHARED src/export_fsm_states_plugin.cpp)
target_include_directories(biped_fsm_plugins PUBLIC include)
ament_target_dependencies(biped_fsm_plugins 
  rclcpp 
  yasmin 
  pluginlib
)

# Tell ROS 2 where the plugin XML file is
pluginlib_export_plugin_description_file(yasmin plugins.xml)

# Install the plugin library
install(TARGETS biped_fsm_plugins
  DESTINATION lib/${PROJECT_NAME}
)

# ---------- c++ related -----------
include_directories(include)

add_executable(state_manager_node src/state_manager.cpp)
target_link_libraries(state_manager_node
  iceoryx2-cxx::static-lib-cxx
)
ament_target_dependencies(state_manager_node 
                            rclcpp 
                            yasmin 
                            yasmin_viewer 
                            yasmin_factory
                            sensor_msgs
                            std_msgs
                            geometry_msgs
                            tf2_ros
                            iox2_msgs
                            )


add_executable(test_fsm_shm_node src/test_fsm_shm.cpp)
ament_target_dependencies(test_fsm_shm_node 
                            rclcpp 
                            yasmin 
                            yasmin_viewer 
                            yasmin_factory
                            sensor_msgs
                            std_msgs
                            geometry_msgs
                            tf2_ros
                            ${Boost_LIBRARIES}
                            )

install(TARGETS
  state_manager_node
  test_fsm_shm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

# -------------------------------------

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(biped_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial REQUIRED)
find_package(fmt REQUIRED)
find_package(ros2_fmt_logger REQUIRED)
find_package(spdlog REQUIRED)

add_library(
  biped_hardware_interface
  SHARED
  src/biped_sanpo_hardware_interface_single_threaded.cpp
  src/biped_sanpo_hardware_interface_multi_threaded.cpp
)
target_include_directories(
  biped_hardware_interface
  PUBLIC
  include
)
ament_target_dependencies(
  biped_hardware_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  serial
  fmt
  ros2_fmt_logger
  spdlog
)

# prevent pluginlib from using boost
target_compile_definitions(biped_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface biped_hardware_interface.xml
)

install(
  TARGETS
  biped_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_export_include_directories(
  include
  src
)
ament_export_libraries(
  biped_hardware_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  biped_hardware_interface
  serial
  fmt
  ros2_fmt_logger
  spdlog
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables (ROS nodes)
install(PROGRAMS
  biped_hardware_interface/test_hardware_interface_position_controller_sine_test.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(sine_test_sanpo_interface test/sine_test_sanpo_interface.cpp)
target_include_directories(sine_test_sanpo_interface
  PRIVATE
  include/biped_hardware_interface
)
ament_target_dependencies(sine_test_sanpo_interface 
  serial 
)
target_link_libraries(sine_test_sanpo_interface
  fmt::fmt  
  spdlog::spdlog
)

add_executable(test_sanpo_interface test/test_sanpo_interface.cpp)
target_include_directories(test_sanpo_interface
  PRIVATE
  include/biped_hardware_interface
)
ament_target_dependencies(test_sanpo_interface 
  serial 
)
target_link_libraries(test_sanpo_interface
  fmt::fmt  
  spdlog::spdlog
)


# Install the executable so you can run it
install(TARGETS 
        sine_test_sanpo_interface
        test_sanpo_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()

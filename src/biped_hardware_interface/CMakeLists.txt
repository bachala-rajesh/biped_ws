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

add_library(
  biped_hardware_interface
  SHARED
  src/biped_system.cpp
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
)
# prevent pluginlib from using boost
target_compile_definitions(biped_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface biped_hardware_interface.xml)

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

# option(BUILD_TESTING "Build the testing tree." OFF)



ament_export_include_directories(
  include
)
ament_export_libraries(
  biped_hardware_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

add_executable(serial_example src/serial_example.cpp)
ament_target_dependencies(serial_example 
  serial 
)

add_executable(serial_debug src/serial_debug.cpp)
ament_target_dependencies(serial_debug 
  serial 
)

# Install the executable so you can run it
install(TARGETS 
          serial_example
          serial_debug
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

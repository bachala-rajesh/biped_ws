cmake_minimum_required(VERSION 3.8)
project(test_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

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

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (ROS nodes)
install(PROGRAMS
  test_pkg/dummy_imu_shared_memory.py
  test_pkg/read_imu_shared_memory_cpp.py
  test_pkg/read_imu_shared_memory_python.py
  test_pkg/robot_states_fsm_python.py
  DESTINATION lib/${PROJECT_NAME}
)

# add c++ executables
add_executable(dummy_imu_shared_memory_node src/dummy_imu_shared_memory.cpp)
ament_target_dependencies(dummy_imu_shared_memory_node rclcpp sensor_msgs)
target_link_libraries(dummy_imu_shared_memory_node 
  ${Boost_LIBRARIES}
  rt
  pthread
)

add_executable(read_imu_shared_memory_node src/read_imu_shared_memory.cpp)
ament_target_dependencies(read_imu_shared_memory_node rclcpp sensor_msgs)
target_link_libraries(read_imu_shared_memory_node 
  ${Boost_LIBRARIES}
  rt
  pthread
)



install(TARGETS dummy_imu_shared_memory_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS read_imu_shared_memory_node DESTINATION lib/${PROJECT_NAME})


# Install files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()

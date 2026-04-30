cmake_minimum_required(VERSION 3.8)
project(biped_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(iceoryx2-cxx REQUIRED)
find_package(iox2_msgs REQUIRED)



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

# C++ executables
add_executable(mujoco_imu_iox2_node src/mujoco_imu_iox2_node.cpp)
ament_target_dependencies(mujoco_imu_iox2_node
             rclcpp
             sensor_msgs
             iox2_msgs
)
target_link_libraries(mujoco_imu_iox2_node
  iceoryx2-cxx::static-lib-cxx
)

add_executable(mujoco_imu_iox2_subscriber_node src/mujoco_imu_iox2_subscriber_node.cpp)
ament_target_dependencies(mujoco_imu_iox2_subscriber_node
             iox2_msgs
)
target_link_libraries(mujoco_imu_iox2_subscriber_node
  iceoryx2-cxx::static-lib-cxx
)

install(TARGETS mujoco_imu_iox2_node mujoco_imu_iox2_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables (ROS nodes)
install(PROGRAMS
  biped_bringup/gui_sliders_biped_joints_control.py
  biped_bringup/reset_world_client.py
  
  DESTINATION lib/${PROJECT_NAME}
)

# Install files
install(DIRECTORY
  launch
  rviz
  config
  world
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

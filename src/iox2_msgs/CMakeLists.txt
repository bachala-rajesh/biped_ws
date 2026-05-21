cmake_minimum_required(VERSION 3.8)
project(iox2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(iceoryx2-cxx REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



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

#  C++ executables
add_executable(test_iox2_imu src/test_iox2_imu.cpp)
target_include_directories(test_iox2_imu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(test_iox2_imu
  rclcpp
)
target_link_libraries(test_iox2_imu
  iceoryx2-cxx::static-lib-cxx
)



# install C++ executables
install(TARGETS
  test_iox2_imu
  DESTINATION lib/${PROJECT_NAME}
)

# install headers so downstream packages can use them
install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()

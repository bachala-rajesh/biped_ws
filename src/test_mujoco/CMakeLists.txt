cmake_minimum_required(VERSION 3.8)
project(test_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# setting path for finding torch
set(CMAKE_PREFIX_PATH "/usr/local/lib/python3.10/dist-packages/torch/share/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Torch REQUIRED)
find_package(Eigen3 REQUIRED)
find_library(MUJOCO_LIBRARY
  NAMES mujoco
  PATHS /usr/local/lib
  REQUIRED
)

find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  PATHS /usr/local/include
  REQUIRED
)
find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  PATHS /usr/local/lib
  REQUIRED
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

# Install Python modules
ament_python_install_package(${PROJECT_NAME})


# Add node mujoco_node_torch
add_executable(mujoco_node_torch 
                    src/mujoco_node_torch.cpp
                    src/mujoco_viewer.cpp
                    src/mujoco_sim.cpp
                    src/observation_builder.cpp
                    src/torch_policy.cpp
              )
target_include_directories(mujoco_node_torch 
          PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${MUJOCO_INCLUDE_DIR}
            ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(mujoco_node_torch
            "rclcpp"
            "ament_index_cpp"
)
target_link_libraries(mujoco_node_torch 
            ${MUJOCO_LIBRARY} 
            glfw OpenGL::GL
            ${TORCH_LIBRARIES}
          )
set_target_properties(mujoco_node_torch PROPERTIES
            INSTALL_RPATH_USE_LINK_PATH TRUE
)


# Add node mujoco_node_torch
add_executable(mujoco_node_onnx 
                    src/mujoco_node_onnx.cpp
                    src/mujoco_viewer.cpp
                    src/mujoco_sim.cpp
                    src/observation_builder.cpp
                    src/onnx_policy.cpp
              )
target_include_directories(mujoco_node_onnx 
          PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${MUJOCO_INCLUDE_DIR}
            ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(mujoco_node_onnx
            "rclcpp"
            "ament_index_cpp"
)
target_link_libraries(mujoco_node_onnx 
            ${MUJOCO_LIBRARY} 
            glfw OpenGL::GL
            ${ONNXRUNTIME_LIBRARY}
          )
set_target_properties(mujoco_node_onnx PROPERTIES
            INSTALL_RPATH_USE_LINK_PATH TRUE
)


install(TARGETS 
      mujoco_node_torch
      mujoco_node_onnx
  DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY
            mujoco_xml
            models
        DESTINATION share/${PROJECT_NAME}
)

ament_package()

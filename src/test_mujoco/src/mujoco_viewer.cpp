#include "test_mujoco/mujoco_viewer.hpp"
#include <stdexcept>
#include <iostream>
#include <string>
#include <GLFW/glfw3.h>

namespace test_mujoco {

MujocoViewer::MujocoViewer(mjModel* model, mjData* data, int width, int height, const char* title)
    : model_(model), data_(data) {
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    // create window
    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    glfwMakeContextCurrent(window_); 
    glfwSwapInterval(1); // enable vsync

    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&con_);

    mjv_makeScene(model, &scene_, 2000);
    mjr_makeContext(model, &con_, mjFONTSCALE_150);

    // camera default view
    cam_.lookat[0] = 0.0; cam_.lookat[1] = 0.0; cam_.lookat[2] = 0.0;
    cam_.distance = 3.0;
    cam_.azimuth = 135;
    cam_.elevation = -20;
}

MujocoViewer::~MujocoViewer() {
    mjv_freeScene(&scene_);
    mjr_freeContext(&con_);
    if (window_) {
        glfwDestroyWindow(window_);
    }
    glfwTerminate();
}

bool MujocoViewer::is_open() const {
    return window_ && !glfwWindowShouldClose(window_);
}

void MujocoViewer::render() {
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    
    mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scene_);
    mjr_render(viewport, &scene_, &con_);

    glfwSwapBuffers(window_);
    glfwPollEvents();
}

void MujocoViewer::set_camera(double lookat_x,
                               double lookat_y,
                               double lookat_z,
                               double distance,
                               double azimuth,
                               double elevation) {
    cam_.lookat[0] = lookat_x; cam_.lookat[1] = lookat_y; cam_.lookat[2] = lookat_z;
    cam_.distance = distance;
    cam_.azimuth = azimuth;
    cam_.elevation = elevation;
}

} // namespace test_mujoco

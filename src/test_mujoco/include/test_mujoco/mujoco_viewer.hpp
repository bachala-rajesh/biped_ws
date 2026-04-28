#pragma once

#include <mujoco/mujoco.h>

struct GLFWwindow;

namespace test_mujoco {

class MujocoViewer {
public:
    MujocoViewer(mjModel* model, mjData* data, int width = 1200, int height = 900, const char* title= "biped");
    ~MujocoViewer();

    MujocoViewer(const MujocoViewer&) = delete;
    MujocoViewer& operator=(const MujocoViewer&) = delete;    

    bool is_open() const;
    void render();

    void set_camera(double lookat_x,
                     double lookat_y,
                     double lookat_z,
                     double distance,
                     double azimuth,
                     double elevation);

private:
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
    GLFWwindow* window_ = nullptr;

    mjvCamera cam_;
    mjvScene scene_;
    mjvOption opt_;
    mjrContext con_;
};

} // namespace test_mujoco  

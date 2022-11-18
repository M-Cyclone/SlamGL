#pragma once
#include <atomic>
#include <memory>
#include <thread>

#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include <sophus/se3.hpp>

#include "core/SlamKernel.h"

#include "Camera.h"

class Renderer
{
public:
    Renderer(GLFWwindow* window, std::shared_ptr<Camera> camera);
    Renderer(const Renderer&)            = delete;
    Renderer& operator=(const Renderer&) = delete;
    ~Renderer() noexcept;

    void shutdown() { m_is_running = false; }

    bool isRunning() const { return m_is_running; }


    void setPointCloud(std::vector<SlamKernel::PointVertex> data)
    {
        std::unique_lock<std::shared_mutex> lock(m_data_mutex);
        m_vertices = std::move(data);
    }

    void setKeyFrames(std::vector<Sophus::SE3f> data)
    {
        std::unique_lock<std::shared_mutex> lock(m_data_mutex);
        m_keyframe_poses = std::move(data);
    }

private:
    void renderMain();

private:
    GLFWwindow*             m_window = nullptr;
    std::shared_ptr<Camera> m_camera;
    std::thread             m_render_thread;

    mutable std::shared_mutex            m_data_mutex;
    std::vector<SlamKernel::PointVertex> m_vertices;
    std::vector<Sophus::SE3f>            m_keyframe_poses;

    std::atomic_bool m_is_running = true;
};
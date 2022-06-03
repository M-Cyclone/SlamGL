#pragma once
#include <memory>
#include <thread>
#include <atomic>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "core/SlamKernel.h"

#include "PointCloud.h"
#include "Camera.h"

class Renderer
{
public:
    Renderer(GLFWwindow* window, std::shared_ptr<PointCloud> pc, std::shared_ptr<Camera> camera);
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    ~Renderer() noexcept;

    void shutdown()
    {
        m_is_running = false;
    }

    bool isRunning() const { return m_is_running; }

private:
    void renderMain();

private:
    GLFWwindow* m_window = nullptr;
    std::shared_ptr<PointCloud> m_pc;
    std::shared_ptr<Camera> m_camera;
    std::thread m_render_thread;

    std::atomic_bool m_is_running = true;
};
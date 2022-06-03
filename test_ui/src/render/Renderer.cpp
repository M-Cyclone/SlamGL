#include "Renderer.h"
#include <chrono>
#include <cstdint>
#include <shared_mutex>
#include <string>

#include <stb/stb_image.h>
#include <stb/stb_image_write.h>
#include <thread>

#include "core/SlamKernel.h"
#include "glad/glad.h"
#include "glm/fwd.hpp"
#include "render/Shader.h"
#include "sophus/se3.hpp"
#include "utils/Log.h"
#include "utils/Timer.h"

extern std::string* p_shader_folder_path;

Renderer::Renderer(GLFWwindow* window, std::shared_ptr<Camera> camera)
    : m_window(window)
    , m_camera(camera)
{
    m_render_thread = std::thread(&Renderer::renderMain, this);
}

Renderer::~Renderer() noexcept
{
    m_render_thread.join();
}

void Renderer::renderMain()
{
    using Vertex = SlamKernel::PointVertex;

    glfwMakeContextCurrent(m_window);


    {
        bool check = gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
        APP_ASSERT(check, "Failed to initialize glad.");
    }

    
    glfwSetWindowUserPointer(m_window, this);
    glfwSetWindowCloseCallback(m_window, [](GLFWwindow* window)
    {
        Renderer& renderer = *static_cast<Renderer*>(glfwGetWindowUserPointer(window));
        renderer.shutdown();
    });

    glPointSize(3.0f);
    glLineWidth(1.0f);


    int wnd_width;
    int wnd_height;
    glfwGetWindowSize(m_window, &wnd_width, &wnd_height);


    std::string pc_vert_path = *p_shader_folder_path + "/point_cloud.vert";
    std::string pc_frag_path = *p_shader_folder_path + "/point_cloud.frag";
    Shader point_cloud_shader(pc_vert_path, pc_frag_path);


    std::string kf_vert_path = *p_shader_folder_path + "/keyframe.vert";
    std::string kf_frag_path = *p_shader_folder_path + "/keyframe.frag";
    Shader keyframe_shader(kf_vert_path, kf_frag_path);


    Timer timer;
    while(m_is_running)
    {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        
        // draw point cloud
        std::vector<Vertex> point_cloud_vertices;
        {
            std::shared_lock<std::shared_mutex> lock(m_data_mutex);
            point_cloud_vertices = m_vertices;
        }
        if(!point_cloud_vertices.empty())
        {
            point_cloud_shader.bind();
            point_cloud_shader.setValue("u_proj", m_camera->getProj());
            point_cloud_shader.setValue("u_view", m_camera->getView());

            uint32_t vao = 0;
            glGenVertexArrays(1, &vao);
            glBindVertexArray(vao);

            uint32_t vbo = 0;
            glGenBuffers(1, &vbo);
            glBindBuffer(GL_ARRAY_BUFFER, vbo);

            glBufferData(GL_ARRAY_BUFFER
                , sizeof(Vertex) * point_cloud_vertices.size()
                , point_cloud_vertices.data()
                , GL_STATIC_DRAW
                );
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const void*)offsetof(Vertex, position));
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const void*)offsetof(Vertex, color));
            glEnableVertexAttribArray(1);


            glDrawArrays(GL_POINTS, 0, point_cloud_vertices.size());

            glDeleteBuffers(1, &vbo);
            glDeleteVertexArrays(1, &vao);
        }
        

        // // draw keyframes
        // std::vector<glm::vec3> kf_positions;
        // {
        //     std::shared_lock<std::shared_mutex> lock(m_data_mutex);
        //     for(const auto& p : m_keyframe_poses)
        //     {
        //         auto t = p.translation();
        //         kf_positions.emplace_back(t.x(), t.y(), t.z());
        //     }
        // }
        // if(!kf_positions.empty())
        // {
        //     keyframe_shader.bind();
        //     keyframe_shader.setValue("u_proj", m_camera->getProj());
        //     keyframe_shader.setValue("u_view", m_camera->getView());

        //     uint32_t vao = 0;
        //     glGenVertexArrays(1, &vao);
        //     glBindVertexArray(vao);

        //     uint32_t vbo = 0;
        //     glGenBuffers(1, &vbo);
        //     glBindBuffer(GL_ARRAY_BUFFER, vbo);

        //     glBufferData(GL_ARRAY_BUFFER
        //         , sizeof(glm::vec3) * kf_positions.size()
        //         , kf_positions.data()
        //         , GL_STATIC_DRAW
        //         );
        //     glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (const void*)0);
        //     glEnableVertexAttribArray(0);


        //     glDrawArrays(GL_LINE_STRIP, 0, kf_positions.size());

        //     glDeleteBuffers(1, &vbo);
        //     glDeleteVertexArrays(1, &vao);
        // }


        // record video
        {
            stbi_flip_vertically_on_write(true);
            static size_t img_count = 0;
            std::vector<stbi_uc> image(wnd_width * wnd_height * 4);
            glReadPixels(0, 0, wnd_width, wnd_height, GL_RGBA, GL_UNSIGNED_BYTE, image.data());
            
            std::string name = "/media/misaka/MISAKA/Video/" + std::to_string(img_count++) + ".png";
            stbi_write_png(name.c_str(), wnd_width, wnd_height, 4, image.data(), 4 * wnd_width);
        }


        float dt = timer.mark() * 1e-6; // timer gives the time in nanoseconds.
        if(dt < Timer::k_frame_delte_time)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(size_t(Timer::k_frame_delte_time - dt)));
            dt = Timer::k_frame_delte_time;
        }
        glfwSwapBuffers(m_window);
    }
}
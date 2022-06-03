#include "Renderer.h"
#include <chrono>
#include <cstdint>
#include <string>

#include <stb/stb_image.h>
#include <stb/stb_image_write.h>
#include <thread>

#include "render/Shader.h"
#include "utils/Log.h"
#include "utils/Timer.h"

extern std::string* p_shader_folder_path;

Renderer::Renderer(GLFWwindow* window, std::shared_ptr<PointCloud> pc, std::shared_ptr<Camera> camera)
    : m_window(window)
    , m_pc(pc)
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
    using Vertex = decltype(m_pc->getData())::value_type;

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


    int wnd_width;
    int wnd_height;
    glfwGetWindowSize(m_window, &wnd_width, &wnd_height);


    std::string pc_vert_path = *p_shader_folder_path + "/point_cloud.vert";
    std::string pc_frag_path = *p_shader_folder_path + "/point_cloud.frag";
    Shader point_cloud_shader(pc_vert_path, pc_frag_path);


    Timer timer;
    while(m_is_running)
    {
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        
        std::vector<Vertex> point_cloud_vertices = m_pc->getData();
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

            // These codes are used to record the runtime screen shots.
            // {
            //     static size_t img_idx = 0;
            //     std::string name = "/media/misaka/MISAKA/Programs/Orbslam3_UI/video/" + std::to_string(img_idx) + ".png";
            //     std::vector<stbi_uc> img(wnd_width * wnd_height * 4 * sizeof(stbi_uc));
            //     std::cout << "Starts read buffer" << std::endl;
            //     glReadPixels(0, 0, wnd_width, wnd_height, GL_RGBA, GL_UNSIGNED_BYTE, img.data());
            //     std::cout << "Ends read buffer" << std::endl;
            //     stbi_write_png(name.c_str(), wnd_width, wnd_height, 4, img.data(), wnd_width * 4);

            //     ++img_idx;
            // }

            float dt = timer.mark() * 1e-6; // timer gives the time in nanoseconds.
            if(dt < Timer::k_frame_delte_time)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(size_t(Timer::k_frame_delte_time - dt)));
                dt = Timer::k_frame_delte_time;
            }
        }
        

        glfwSwapBuffers(m_window);
    }
}
#include "Renderer.h"
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <shared_mutex>

#include <stb/stb_image.h>
#include <stb/stb_image_write.h>

#include <imgui/backends/imgui_impl_glfw.h>
#include <imgui/backends/imgui_impl_opengl3.h>
#include <imgui/imgui.h>

#include "core/SlamKernel.h"
#include "render/Shader.h"
#include "utils/Log.h"
#include "utils/Timer.h"

extern const std::string* p_shader_folder_path;

Renderer::Renderer(GLFWwindow* window, std::shared_ptr<Camera> camera)
    : m_window(window), m_camera(camera)
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
    glfwSetWindowCloseCallback(m_window,
                               [](GLFWwindow* window)
                               {
                                   Renderer& renderer = *static_cast<Renderer*>(
                                       glfwGetWindowUserPointer(window));
                                   renderer.shutdown();
                               });

    glPointSize(3.0f);
    glLineWidth(1.0f);


    int wnd_width;
    int wnd_height;
    glfwGetWindowSize(m_window, &wnd_width, &wnd_height);


    // IMGUI init
    {
        IMGUI_CHECKVERSION();

        ImGui::CreateContext();
        ImGui::StyleColorsDark();

        ImGuiIO& io    = ImGui::GetIO();
        io.IniFilename = nullptr;

        ImFontConfig font_config;
        font_config.SizePixels = 18.0f;
        io.Fonts->AddFontDefault(&font_config);

        if (!ImGui_ImplGlfw_InitForOpenGL(m_window, true))
        {
            APP_ERROR("Failed to init ImGui's GLFW implementation.");
            throw -1;
        }

        if (!ImGui_ImplOpenGL3_Init("#version 130"))
        {
            APP_ERROR("Failed to init ImGui's GL3 implementation.");
            throw -1;
        }
    }


    std::string pc_vert_path = *p_shader_folder_path + "/point_cloud.vert";
    std::string pc_frag_path = *p_shader_folder_path + "/point_cloud.frag";
    Shader      point_cloud_shader(pc_vert_path, pc_frag_path);


    std::string kf_vert_path = *p_shader_folder_path + "/keyframe.vert";
    std::string kf_frag_path = *p_shader_folder_path + "/keyframe.frag";
    Shader      keyframe_shader(kf_vert_path, kf_frag_path);


    Timer timer;
    while (m_is_running)
    {
        static float dt = Timer::k_frame_delte_time;

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        // data update
        auto mat_proj = m_camera->getProj();
        auto mat_view = m_camera->getView();


        // draw point cloud
        static bool show_point_cloud = true;
        if (show_point_cloud)
        {
            std::vector<Vertex> point_cloud_vertices;
            {
                std::shared_lock<std::shared_mutex> lock(m_data_mutex);
                point_cloud_vertices = m_vertices;
            }
            if (!point_cloud_vertices.empty())
            {
                point_cloud_shader.bind();
                point_cloud_shader.setValue("u_proj", mat_proj);
                point_cloud_shader.setValue("u_view", mat_view);

                uint32_t vao = 0;
                glGenVertexArrays(1, &vao);
                glBindVertexArray(vao);

                uint32_t vbo = 0;
                glGenBuffers(1, &vbo);
                glBindBuffer(GL_ARRAY_BUFFER, vbo);

                glBufferData(GL_ARRAY_BUFFER,
                             sizeof(Vertex) * point_cloud_vertices.size(),
                             point_cloud_vertices.data(),
                             GL_STATIC_DRAW);
                glVertexAttribPointer(0,
                                      3,
                                      GL_FLOAT,
                                      GL_FALSE,
                                      sizeof(Vertex),
                                      (const void*)offsetof(Vertex, position));
                glEnableVertexAttribArray(0);
                glVertexAttribPointer(1,
                                      3,
                                      GL_FLOAT,
                                      GL_FALSE,
                                      sizeof(Vertex),
                                      (const void*)offsetof(Vertex, color));
                glEnableVertexAttribArray(1);


                glDrawArrays(GL_POINTS, 0, point_cloud_vertices.size());

                glDeleteBuffers(1, &vbo);
                glDeleteVertexArrays(1, &vao);
            }
        }


        // draw keyframes
        static bool show_keyframes = true;
        if (show_keyframes)
        {
            std::vector<glm::vec3> kf_positions;
            {
                std::shared_lock<std::shared_mutex> lock(m_data_mutex);
                for (const auto& p : m_keyframe_poses)
                {
                    auto t = p.translation();
                    kf_positions.emplace_back(t.x(), t.y(), t.z());
                }
            }
            if (!kf_positions.empty())
            {
                keyframe_shader.bind();
                keyframe_shader.setValue("u_proj", mat_proj);
                keyframe_shader.setValue("u_view", mat_view);

                uint32_t vao = 0;
                glGenVertexArrays(1, &vao);
                glBindVertexArray(vao);

                uint32_t vbo = 0;
                glGenBuffers(1, &vbo);
                glBindBuffer(GL_ARRAY_BUFFER, vbo);

                glBufferData(GL_ARRAY_BUFFER,
                             sizeof(glm::vec3) * kf_positions.size(),
                             kf_positions.data(),
                             GL_STATIC_DRAW);
                glVertexAttribPointer(0,
                                      3,
                                      GL_FLOAT,
                                      GL_FALSE,
                                      sizeof(glm::vec3),
                                      (const void*)0);
                glEnableVertexAttribArray(0);


                glDrawArrays(GL_LINE_STRIP, 0, kf_positions.size());

                glDeleteBuffers(1, &vbo);
                glDeleteVertexArrays(1, &vao);
            }
        }


        // record video
        static bool record_video = false;
        if (record_video)
        {
            stbi_flip_vertically_on_write(true);
            static size_t        img_count = 0;
            std::vector<stbi_uc> image(wnd_width * wnd_height * 4);
            glReadPixels(0,
                         0,
                         wnd_width,
                         wnd_height,
                         GL_RGBA,
                         GL_UNSIGNED_BYTE,
                         image.data());

            std::string name = "/media/misaka/MISAKA/Video/" +
                               std::to_string(img_count++) + ".png";
            stbi_write_png(name.c_str(),
                           wnd_width,
                           wnd_height,
                           4,
                           image.data(),
                           4 * wnd_width);
        }


        // UI handle
        {
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            ImGui::Text("Current FPS: %.3f", 1000.0f / dt);

            if (ImGui::CollapsingHeader("Render Options"))
            {
                if (ImGui::TreeNode("Point Cloud Options"))
                {
                    if (ImGui::Button("Show / Hide Point Cloud"))
                    {
                        show_point_cloud = !show_point_cloud;
                    }

                    ImGui::TreePop();
                }

                if (ImGui::TreeNode("Key Frame Options"))
                {
                    if (ImGui::Button("Show / Hide Key Frames"))
                    {
                        show_keyframes = !show_keyframes;
                    }

                    ImGui::TreePop();
                }

                if (ImGui::TreeNode("Recording Options"))
                {
                    if (ImGui::Button(record_video ? "Stop Recording"
                                                   : "Start Recording"))
                    {
                        record_video = !record_video;
                    }

                    ImGui::TreePop();
                }
            }

            if (ImGui::Button("Exit"))
            {
                m_is_running = false;
            }


            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }


        dt = timer.mark() * 1e-6;  // timer gives the time in nanoseconds.
        if (dt < Timer::k_frame_delte_time)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                size_t(Timer::k_frame_delte_time - dt)));
            dt = Timer::k_frame_delte_time;
        }
        glfwSwapBuffers(m_window);
    }


    // ImGui exit
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
}
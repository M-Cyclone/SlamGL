#pragma once
#include "glm/fwd.hpp"
#include <mutex>
#include <shared_mutex>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Camera
{
public:
    struct Desc
    {
        float fov = 45.0f;
        float z_near = 0.1f;
        float z_far = 100.0f;
        int width = 1280;
        int height = 720;
    };

public:
    Camera(const Desc& desc);
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    // This function is used to set pose from slam world axis.
    void setPose(const Sophus::SE3f& pose)
    {
        auto translation = pose.translation();
        auto rotation_mat = pose.so3();
        auto right = (rotation_mat * Eigen::Vector3f(1.0f, 0.0f, 0.0f)).normalized();
        auto forward = (rotation_mat * Eigen::Vector3f(0.0f, 0.0f, 1.0f)).normalized();
        auto up = right.cross(forward).normalized();

        std::unique_lock<std::shared_mutex> lock(m_data_mutex);
        m_position = glm::vec3(translation.x(), -translation.y(), -translation.z());
        m_right = glm::vec3(right.x(), -right.y(), -right.z());
        m_forward = glm::vec3(forward.x(), -forward.y(), -forward.z());
        m_up = glm::vec3(up.x(), -up.y(), -up.z());
    }

    glm::mat4 getProj() const
    {
        // proj will use identity matrix as default
        std::shared_lock<std::shared_mutex> lock(m_data_mutex);
        return glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f);
    }

    glm::mat4 getView() const
    {
        std::shared_lock<std::shared_mutex> lock(m_data_mutex);
        return glm::lookAt(m_position, m_position + m_forward, m_up);
    }

private:
    glm::vec3 m_position;
    glm::vec3 m_forward;
    glm::vec3 m_up;
    glm::vec3 m_right;
    float m_fov = 45.0f;
    float m_z_near = 0.1f;
    float m_z_far = 100.0f;
    float m_aspect_radio = 1.0f;

    mutable std::shared_mutex m_data_mutex;
};
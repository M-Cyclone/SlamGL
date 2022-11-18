#pragma once
#include <mutex>
#include <shared_mutex>
#include "glm/ext/matrix_clip_space.hpp"
#include "glm/fwd.hpp"
#include "glm/matrix.hpp"
#include "glm/trigonometric.hpp"

#include <Eigen/Core>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <sophus/se3.hpp>

class Camera
{
public:
    struct Desc
    {
        float fov    = 45.0f;
        float z_near = 0.1f;
        float z_far  = 100.0f;
        int   width  = 1280;
        int   height = 720;
    };

public:
    Camera(const Desc& desc);
    Camera(const Camera&)            = delete;
    Camera& operator=(const Camera&) = delete;

    // This function is used to set pose from slam world axis.
    void setPose(const Sophus::SE3f& pose)
    {
        std::unique_lock<std::shared_mutex> lock(m_data_mutex);
        m_pose = pose;
    }

    glm::mat4 getProj() const
    {
        std::shared_lock<std::shared_mutex> lock(m_data_mutex);
        return glm::perspective(glm::radians(m_fov),
                                m_aspect_radio,
                                m_z_near,
                                m_z_far);
    }

    glm::mat4 getView() const
    {
        static const glm::mat4
            change(1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1);

        std::shared_lock<std::shared_mutex> lock(m_data_mutex);
        auto                                mat_pose = m_pose.matrix();

        glm::mat4 glm_pose(mat_pose(0, 0),
                           mat_pose(1, 0),
                           mat_pose(2, 0),
                           mat_pose(3, 0),
                           mat_pose(0, 1),
                           mat_pose(1, 1),
                           mat_pose(2, 1),
                           mat_pose(3, 1),
                           mat_pose(0, 2),
                           mat_pose(1, 2),
                           mat_pose(2, 2),
                           mat_pose(3, 2),
                           mat_pose(0, 3),
                           mat_pose(1, 3),
                           mat_pose(2, 3),
                           mat_pose(3, 3));
        return change * glm_pose;
    }

private:
    Sophus::SE3f m_pose;
    float        m_fov          = 45.0f;
    float        m_z_near       = 0.1f;
    float        m_z_far        = 100.0f;
    float        m_aspect_radio = 1.0f;

    mutable std::shared_mutex m_data_mutex;
};
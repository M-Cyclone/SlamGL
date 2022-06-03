#include "Camera.h"
#include "render/Camera.h"
#include "sophus/se3.hpp"

Camera::Camera(const Desc& desc)
    : m_position(0.0f, 0.0f, 0.0f)
    , m_forward(0.0f, 0.0f, -1.0f)
    , m_up(0.0f, 1.0f, 0.0f)
    , m_fov(desc.fov)
    , m_z_near(desc.z_near)
    , m_z_far(desc.z_far)
    , m_aspect_radio((float)desc.width / (float)desc.height)
{

}
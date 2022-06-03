#include "Camera.h"
#include "render/Camera.h"

Camera::Camera(const Desc& desc)
    : m_fov(desc.fov)
    , m_z_near(desc.z_near)
    , m_z_far(desc.z_far)
    , m_aspect_radio((float)desc.width / (float)desc.height)
{}
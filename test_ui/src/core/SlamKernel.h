#pragma once
#include <vector>
#include <mutex>
#include <shared_mutex>

#include <glm/glm.hpp>
#include <sophus/se3.hpp>

#include "utils/DataType.h"

class SlamKernel
{
public:
    struct PointVertex
    {
        glm::vec3 position;
        glm::vec3 color;
    };

protected:
    SlamKernel() = default;
    SlamKernel(const SlamKernel&) = delete;
    SlamKernel& operator=(const SlamKernel&) = delete;

public:
    virtual ~SlamKernel() noexcept = default;

    virtual Sophus::SE3f track(const std::vector<ImgData>& imgs, const std::vector<ImuData>& imus) = 0;

    virtual void shutdown() = 0;

    virtual std::vector<PointVertex> getPointCloudVetices() = 0;
    virtual std::vector<Sophus::SE3f> getKeyFramePoses() = 0;
};
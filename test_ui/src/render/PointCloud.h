#pragma once
#include <mutex>
#include <shared_mutex>
#include <vector>

#include "core/SlamKernel.h"

class PointCloud
{
public:
    PointCloud() = default;
    PointCloud(const PointCloud&) = delete;
    PointCloud& operator=(const PointCloud&) = delete;

    void setData(std::vector<SlamKernel::PointVertex> data)
    {
        std::unique_lock<std::shared_mutex> lock(m_data_mutex);
        m_vertices = std::move(data);
    }

    auto getData() const
    {
        std::shared_lock<std::shared_mutex> lock(m_data_mutex);
        return m_vertices;
    }

private:
    mutable std::shared_mutex m_data_mutex;
    std::vector<SlamKernel::PointVertex> m_vertices;
};
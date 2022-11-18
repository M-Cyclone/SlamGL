#pragma once
#include <chrono>

class Timer
{
    using Clock = std::chrono::steady_clock;

public:
    Timer() : m_last(Clock::now()) {}
    Timer(const Timer&)            = delete;
    Timer& operator=(const Timer&) = delete;

    float mark() const noexcept
    {
        const auto old = m_last;
        m_last         = Clock::now();
        return (m_last - old).count();
    }

    float peek() const noexcept { return (Clock::now() - m_last).count(); }

public:
    static constexpr size_t k_frame_rate       = 30;
    static constexpr float  k_frame_delte_time = 1000.0f / k_frame_rate;

private:
    mutable Clock::time_point m_last;
};
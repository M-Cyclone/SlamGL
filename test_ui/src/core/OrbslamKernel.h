#pragma once
#include <memory>
#include <string>

#include <Core/System.h>
#include <Utils/Settings.h>
#include <Feature/ORBVocabulary.h>

#include "SlamKernel.h"
#include "utils/Log.h"

class OrbslamKernel : public SlamKernel
{
public:
    OrbslamKernel(ORB_SLAM3::Settings* settings, ORB_SLAM3::ORBVocabulary* vocabulary);
    OrbslamKernel(const OrbslamKernel&) = delete;
    OrbslamKernel& operator=(const OrbslamKernel&) = delete;

    Sophus::SE3f track(const std::vector<ImgData>& imgs, const std::vector<ImuData>& imus) override;

    void shutdown() override
    {
        m_orb_system.Shutdown();
    }
    
    std::vector<PointVertex> getPointCloudVetices() override;
    std::vector<Sophus::SE3f> getKeyFramePoses() override;

private:
    ORB_SLAM3::System m_orb_system;
};
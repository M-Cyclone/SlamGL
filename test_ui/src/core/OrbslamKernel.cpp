#include "OrbslamKernel.h"
#include <algorithm>
#include <memory>
#include <vector>
#include <unordered_set>

#include <sophus/se3.hpp>
#include <Eigen/Core>

#include <Utils/ImuTypes.h>
#include <Map/MapPoint.h>
#include <Frame/KeyFrame.h>

#include "utils/Log.h"


OrbslamKernel::OrbslamKernel(ORB_SLAM3::Settings* settings, ORB_SLAM3::ORBVocabulary* vocabulary)
    : m_orb_system(vocabulary, settings, static_cast<ORB_SLAM3::System::eSensor>(settings->getSensor()))
{

}

Sophus::SE3f OrbslamKernel::track(const std::vector<ImgData>& imgs, const std::vector<ImuData>& imus)
{
    static size_t idx = 0;
    APP_INFO("Handle new frame start. Frame idx: {0}", idx++);

    float scale = m_orb_system.GetImageScale();
    int new_width = imgs[0].image.cols * scale;
    int new_height = imgs[0].image.rows * scale;
    cv::resize(imgs[0].image, imgs[0].image, cv::Size(new_width, new_height));

    std::vector<ORB_SLAM3::IMU::Point> imu_points;
    for(const auto& imu : imus)
    {
        imu_points.emplace_back(
            imu.a_x,
            imu.a_y,
            imu.a_z,
            imu.w_x,
            imu.w_y,
            imu.w_z,
            imu.time_stamp * 1e-9
        );
    }
    
    Sophus::SE3f pose = m_orb_system.TrackMonocular(imgs[0].image, imgs[0].time_stamp * 1e-9, imu_points);

    APP_INFO("\n");

    return pose;
}

auto OrbslamKernel::getPointCloudVetices() -> std::vector<PointVertex>
{
    auto local_mp = m_orb_system.getAtlas().GetReferenceMapPoints();
    std::unordered_set<ORB_SLAM3::MapPoint*> local_mp_ust(local_mp.begin(), local_mp.end());

    std::vector<PointVertex> pvs;
    for(auto mp : local_mp_ust)
    {
        if(!mp || mp->isBad()) continue;
        auto eigen_pos = mp->GetWorldPos();
        pvs.push_back({glm::vec3(eigen_pos.x(), eigen_pos.y(), eigen_pos.z()), glm::vec3(1.0f, 0.0f, 0.0f)});
    }

    auto all_mp = m_orb_system.getAtlas().GetAllMapPoints();
    for(auto mp : all_mp)
    {
        if(!mp || mp->isBad() || local_mp_ust.find(mp) != local_mp_ust.end()) continue;
        auto eigen_pos = mp->GetWorldPos();
        pvs.push_back({glm::vec3(eigen_pos.x(), eigen_pos.y(), eigen_pos.z()), glm::vec3(1.0f, 1.0f, 1.0f)});
    }

    APP_INFO("Global map point count: {0}. Local map point count: {1}", local_mp.size(), all_mp.size());

    return pvs;
}

auto OrbslamKernel::getKeyFramePoses() -> std::vector<Sophus::SE3f>
{
    std::vector<Sophus::SE3f> poses;

    auto all_kfs = m_orb_system.getAtlas().GetAllKeyFrames();
    std::sort(all_kfs.begin(), all_kfs.end(), [](const ORB_SLAM3::KeyFrame* kf1, const ORB_SLAM3::KeyFrame* kf2)
    {
        return kf1->mnFrameId < kf2->mnFrameId;
    });
    for(auto kf : all_kfs)
    {
        if(!kf || kf->isBad()) continue;

        poses.push_back(kf->GetPoseInverse());
    }

    APP_INFO("Global keyframe count: {0}", all_kfs.size());

    return poses;
}
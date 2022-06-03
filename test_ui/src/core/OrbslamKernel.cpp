#include "OrbslamKernel.h"
#include <memory>
#include <vector>
#include <unordered_set>

#include <Eigen/Core>

#include <Utils/ImuTypes.h>

#include "Map/MapPoint.h"
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
    APP_INFO("Resize image. New Size: ({0}, {1})", new_width, new_height);
    cv::resize(imgs[0].image, imgs[0].image, cv::Size(new_width, new_height));

    APP_INFO("Reconstruct imu data.");
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
    APP_INFO("Imu data count: {0}", imu_points.size());

    APP_INFO("ORBSLAM tracking new frame data.");
    Sophus::SE3f pose = m_orb_system.TrackMonocular(imgs[0].image, imgs[0].time_stamp * 1e-9, imu_points);
    APP_INFO("Handle new frame end.");

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

    return pvs;
}
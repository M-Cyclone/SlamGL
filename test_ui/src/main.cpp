#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <opencv2/core/hal/interface.h>
#include <unordered_map>
#include <unordered_set>

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <Eigen/Core>
#include <opencv2/core/types.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <Feature/ORBVocabulary.h>
#include <Map/MapPoint.h>
#include <Utils/Settings.h>

#include "Eigen/src/Core/Matrix.h"
#include "core/OrbslamKernel.h"
#include "render/Camera.h"
#include "render/Renderer.h"
#include "sophus/se3.hpp"
#include "utils/DataLoader.h"
#include "utils/Log.h"
#include "utils/Timer.h"

const std::string* p_shader_folder_path = nullptr;

namespace helper
{

static ::ORB_SLAM3::Settings::SettingDesc getDefaultSettingDesc()
{
    static ::ORB_SLAM3::Settings::SettingDesc settingDesc;
    static bool                               init = false;

    if (!init)
    {
        settingDesc.sensor = ORB_SLAM3::System::eSensor::IMU_MONOCULAR;
        settingDesc.cameraInfo.cameraType =
            ORB_SLAM3::Settings::CameraType::PinHole;
        settingDesc.cameraInfo.fx       = 458.654f;
        settingDesc.cameraInfo.fy       = 457.296f;
        settingDesc.cameraInfo.cx       = 367.215f;
        settingDesc.cameraInfo.cy       = 248.375f;
        settingDesc.distortion          = { -0.28340811f,
                                            0.07395907f,
                                            0.00019359f,
                                            1.76187114e-05f };
        settingDesc.imageInfo.width     = 752;
        settingDesc.imageInfo.height    = 480;
        settingDesc.imageInfo.newWidth  = 752;
        settingDesc.imageInfo.newHeight = 480;
        settingDesc.imageInfo.fps       = 20;
        settingDesc.imageInfo.bRGB      = true;
        settingDesc.imuInfo.noiseGyro   = 1.7e-4f;
        settingDesc.imuInfo.noiseAcc    = 2.0000e-3f;
        settingDesc.imuInfo.gyroWalk    = 1.9393e-05f;
        settingDesc.imuInfo.accWalk     = 3.0000e-03f;
        settingDesc.imuInfo.frequency   = 200.0f;
        settingDesc.imuInfo.cvTbc =
            static_cast<cv::Mat>(cv::Mat_<float>(4, 4) << 0.0148655429818f,
                                 -0.999880929698f,
                                 0.00414029679422f,
                                 -0.0216401454975f,
                                 0.999557249008f,
                                 0.0149672133247f,
                                 0.025715529948f,
                                 -0.064676986768f,
                                 -0.0257744366974f,
                                 0.00375618835797f,
                                 0.999660727178f,
                                 0.00981073058949f,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 1.0f);
        settingDesc.imuInfo.bInsertKFsWhenLost   = true;
        settingDesc.orbInfo.nFeatures            = 1000;
        settingDesc.orbInfo.scaleFactor          = 1.2f;
        settingDesc.orbInfo.nLevels              = 8;
        settingDesc.orbInfo.initThFAST           = 20;
        settingDesc.orbInfo.minThFAST            = 7;
        settingDesc.viewerInfo.keyframeSize      = 0.05f;
        settingDesc.viewerInfo.keyframeLineWidth = 1.0f;
        settingDesc.viewerInfo.graphLineWidth    = 0.9f;
        settingDesc.viewerInfo.pointSize         = 2.0f;
        settingDesc.viewerInfo.cameraSize        = 0.08f;
        settingDesc.viewerInfo.cameraLineWidth   = 3.0f;
        settingDesc.viewerInfo.viewPointX        = 0.0f;
        settingDesc.viewerInfo.viewPointY        = -0.7f;
        settingDesc.viewerInfo.viewPointZ        = -3.5f;
        settingDesc.viewerInfo.viewPointF        = 500.0f;
        settingDesc.viewerInfo.imageViewerScale  = 1.0f;

        init = true;
    }

    return settingDesc;
}

}  // namespace helper

int main(int argc, char** argv)
{
    Log::init();


    if (argc < 4)
    {
        APP_ERROR("Usage: test_new_ui [Path to vocabulary] [Path to data "
                  "folder] [Path to shader folder]");
        return -1;
    }


    const std::string vocabulary_path(argv[1]);
    const std::string data_folder_path(argv[2]);
    const std::string shader_folder_path(argv[3]);
    p_shader_folder_path = &shader_folder_path;


    int         window_width  = 1270;
    int         window_height = 720;
    const char* window_name   = "SlamUI";


    try
    {
        if (glfwInit() != GLFW_TRUE)
        {
            APP_ERROR("Failed to init glfw.");
            glfwTerminate();
            return -1;
        }
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);


        EuRoCDataLoader data_loader(data_folder_path);


        APP_INFO("ORBSLAM System initializing, please wait.");
        auto settings =
            new ORB_SLAM3::Settings(helper::getDefaultSettingDesc());
        auto vocabulary = new ORB_SLAM3::ORBVocabulary();
        vocabulary->loadFromTextFile(vocabulary_path);

        auto slam_kernel = new OrbslamKernel(settings, vocabulary);
        APP_INFO("ORBSLAM System has been initialized.");


        GLFWwindow* window = glfwCreateWindow(window_width,
                                              window_height,
                                              window_name,
                                              nullptr,
                                              nullptr);
        if (!window)
        {
            APP_ERROR("Failed to create GLFWwindow.");
            glfwTerminate();
            return -1;
        }


        Camera::Desc desc;
        auto         camera   = std::make_shared<Camera>(desc);
        auto         renderer = new Renderer(window, camera);


        Timer timer;
        while (renderer->isRunning())
        {
            if (data_loader.hasNextData())
            {
                auto [imgs, imus] = data_loader.getNextData();
                Sophus::SE3f pose = slam_kernel->track(imgs, imus);
                camera->setPose(pose);

                renderer->setPointCloud(slam_kernel->getPointCloudVetices());
                renderer->setKeyFrames(slam_kernel->getKeyFramePoses());
            }

            glfwPollEvents();

            float dt = timer.mark() * 1e-6;
            if (dt < Timer::k_frame_delte_time)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(
                    size_t(Timer::k_frame_delte_time - dt)));
                dt = Timer::k_frame_delte_time;
            }
        }


        // Do my things
        {
            struct KeyFrameInfo
            {
                size_t frame_idx;

                // frame / camera pose
                Sophus::SE3f pose;  // Tcw

                // imu data
                Eigen::Vector3f velocity;  // Vwb

                // imu bias
                Eigen::Vector3f bias_a;
                Eigen::Vector3f bias_w;

                // imu calib
                Sophus::SE3f                    imu_Twb;
                Sophus::SO3f                    imu_rotation;
                Eigen::Vector3f                 imu_position;
                Eigen::DiagonalMatrix<float, 6> imu_cov;
                Eigen::DiagonalMatrix<float, 6> imu_cov_walk;

                std::vector<std::pair<float, float>> keypoint_coords;
                std::vector<size_t>                  map_point_indices;

                std::string toString() const
                {
                    size_t count = keypoint_coords.size();

                    std::ostringstream oss;
                    oss << frame_idx << "    Frame Index"
                        << "\n"
                        << pose.log().transpose() << "    Frame Pose"
                        << "\n"
                        << velocity.transpose() << "    Vwb"
                        << "\n"
                        << bias_a.transpose() << "    bias a"
                        << "\n"
                        << bias_w.transpose() << "    bias w"
                        << "\n"
                        << imu_Twb.log().transpose() << "    Twb"
                        << "\n"
                        << imu_rotation.log().transpose() << " "
                        << imu_position.transpose()
                        << "    imu rotation / position"
                        << "\n"
                        << imu_cov.diagonal().transpose() << "    imu cov"
                        << "\n"
                        << imu_cov_walk.diagonal().transpose()
                        << "    imu cov walk"
                        << "\n"
                        << count << "    key point count"
                        << "\n";

                    for (size_t i = 0; i < count; ++i)
                    {
                        oss << keypoint_coords[i].first << " "
                            << keypoint_coords[i].second << " "
                            << map_point_indices[i] << "\n";
                    }

                    oss << "\n";

                    return oss.str();
                }
            };


            std::vector<KeyFrameInfo>                        kf_infos;
            std::unordered_map<ORB_SLAM3::MapPoint*, size_t> map_points_ump;

            auto keyframes = slam_kernel->getKeyFrames();
            for (auto kf : keyframes)
            {
                auto keypoints = kf->mvKeysUn;
                auto mappoints = kf->GetMapPointMatches();

                KeyFrameInfo kfi;
                kfi.frame_idx    = kf->mnFrameId;
                kfi.pose         = kf->GetPose();
                kfi.velocity     = kf->GetVelocity();
                kfi.bias_a       = kf->GetAccBias();
                kfi.bias_w       = kf->GetGyroBias();
                kfi.imu_Twb      = kf->GetImuPose();
                kfi.imu_position = kf->GetImuPosition();
                kfi.imu_rotation = kf->GetImuRotation();
                kfi.imu_cov      = kf->mImuCalib.Cov;
                kfi.imu_cov_walk = kf->mImuCalib.CovWalk;


                const size_t count = keypoints.size();
                if (mappoints.size() != count)
                {
                    APP_ERROR("Invalid data.");
                    throw(-1);
                }

                for (size_t i = 0; i < count; ++i)
                {
                    auto mp = mappoints[i];
                    if (!mp || mp->isBad()) continue;

                    if (map_points_ump.find(mp) == map_points_ump.end())
                    {
                        size_t next_idx    = map_points_ump.size();
                        map_points_ump[mp] = next_idx;
                    }

                    kfi.keypoint_coords.emplace_back(keypoints[i].pt.x,
                                                     keypoints[i].pt.y);
                    kfi.map_point_indices.push_back(map_points_ump[mp]);
                }

                kf_infos.push_back(std::move(kfi));
            }

            std::vector<std::pair<ORB_SLAM3::MapPoint*, size_t>>
                map_points_list(map_points_ump.begin(), map_points_ump.end());
            std::sort(map_points_list.begin(),
                      map_points_list.end(),
                      [](const auto& a, const auto& b)
                      {
                          return a.second < b.second;
                      });


            {
                std::ofstream map_points_file("./map_points.data");
                for (const auto& [mp, idx] : map_points_list)
                {
                    auto pos = mp->GetWorldPos();
                    map_points_file << pos.x() << " " << pos.y() << " "
                                    << pos.z() << "\n";
                }
            }


            std::sort(kf_infos.begin(),
                      kf_infos.end(),
                      [](const KeyFrameInfo& k1, const KeyFrameInfo& k2)
                      {
                          return k1.frame_idx < k2.frame_idx;
                      });
            {
                std::ofstream keyframe_file("./key_frame.data");
                for (const auto& kfi : kf_infos)
                {
                    const size_t count = kfi.keypoint_coords.size();
                    if (count != kfi.map_point_indices.size())
                    {
                        APP_ERROR("Invalid kfi data.");
                        throw(-1);
                    }

                    keyframe_file << kfi.toString();
                }
            }
        }


        slam_kernel->shutdown();
        APP_INFO("ORBSLAM System shutdown successfully.");

        delete slam_kernel;
        APP_INFO("ORBSLAM System has been destroyed successfully.");
        delete vocabulary;
        delete settings;
        delete renderer;
        APP_INFO("Renderer has been destroyed successfully.");

        glfwDestroyWindow(window);
        glfwTerminate();
        APP_INFO("GLFW terminate successfully.");
    }
    catch (...)
    {
        APP_ERROR("Exception Caught.");
    }

    system("pause");
}
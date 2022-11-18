/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */


#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>

#include <opencv2/core/core.hpp>

#include <Core/System.h>
#include <Utils/ImuTypes.h>

using namespace std;

void LoadImages(const string&   strImagePath,
                const string&   strPathTimes,
                vector<string>& vstrImages,
                vector<double>& vTimeStamps);

void LoadIMU(const string&        strImuPath,
             vector<double>&      vTimeStamps,
             vector<cv::Point3f>& vAcc,
             vector<cv::Point3f>& vGyro);

double ttrack_tot = 0;
int    main(int argc, char* argv[])
{
    if (argc < 5)
    {
        cerr << endl
             << "Usage: ./mono_inertial_euroc path_to_vocabulary "
                "path_to_settings path_to_sequence_folder_1 "
                "path_to_times_file_1 (path_to_image_folder_2 "
                "path_to_times_file_2 ... path_to_image_folder_N "
                "path_to_times_file_N) "
             << endl;
        return 1;
    }

    const int num_seq = (argc - 3) / 2;
    cout << "num_seq = " << num_seq << endl;
    bool   bFileName = (((argc - 3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc - 1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int                         seq;
    vector<vector<string>>      vstrImageFilenames;
    vector<vector<double>>      vTimestampsCam;
    vector<vector<cv::Point3f>> vAcc, vGyro;
    vector<vector<double>>      vTimestampsImu;
    vector<int>                 nImages;
    vector<int>                 nImu;
    vector<int>                 first_imu(num_seq, 0);

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq < num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2 * seq) + 3]);
        string pathTimeStamps(argv[(2 * seq) + 4]);

        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathImu  = pathSeq + "/mav0/imu0/data.csv";

        LoadImages(pathCam0,
                   pathTimeStamps,
                   vstrImageFilenames[seq],
                   vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if ((nImages[seq] <= 0) || (nImu[seq] <= 0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq
                 << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start
        // first

        while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--;  // first imu measurement to be considered
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames.
    // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,
    // true);
    ORB_SLAM3::Settings::SettingDesc settingDesc;
    settingDesc.sensor = ORB_SLAM3::System::eSensor::IMU_MONOCULAR;
    settingDesc.cameraInfo.cameraType =
        ORB_SLAM3::Settings::CameraType::PinHole;
    settingDesc.cameraInfo.fx       = 458.654f;
    settingDesc.cameraInfo.fy       = 457.296f;
    settingDesc.cameraInfo.cx       = 367.215f;
    settingDesc.cameraInfo.cy       = 248.375f;
    settingDesc.distortion->k1      = -0.28340811f;
    settingDesc.distortion->k2      = 0.07395907f;
    settingDesc.distortion->p1      = 0.00019359f;
    settingDesc.distortion->p2      = 1.76187114e-05f;
    settingDesc.imageInfo.width     = 752;
    settingDesc.imageInfo.height    = 480;
    settingDesc.imageInfo.newWidth  = 600;
    settingDesc.imageInfo.newHeight = 350;
    settingDesc.imageInfo.fps       = 60;
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
    auto settings = new ORB_SLAM3::Settings(settingDesc);

    std::cout << "Start loading vocabulary from " << argv[1] << std::endl;
    auto vocabulary = new ORB_SLAM3::ORBVocabulary();
    vocabulary->loadFromTextFile(argv[1]);
    std::cout << "Vocabulary loaded." << std::endl;

    ORB_SLAM3::System SLAM(
        vocabulary,
        settings,
        static_cast<const ORB_SLAM3::System::eSensor>(settingDesc.sensor));

    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track  = 0.f;

    int proccIm = 0;
    for (seq = 0; seq < num_seq; seq++)
    {
        // Main loop
        cv::Mat                       im;
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        proccIm = 0;
        for (int ni = 0; ni < nImages[seq]; ni++, proccIm++)
        {
            // Read image from file
            im = cv::imread(vstrImageFilenames[seq][ni],
                            cv::IMREAD_UNCHANGED);  // CV_LOAD_IMAGE_UNCHANGED);

            double tframe = vTimestampsCam[seq][ni];

            if (im.empty())
            {
                cerr << endl
                     << "Failed to load image at: "
                     << vstrImageFilenames[seq][ni] << endl;
                return 1;
            }

            if (imageScale != 1.f)
            {
                int width  = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
            }

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if (ni > 0)
            {
                // cout << "t_cam " << tframe << endl;

                while (vTimestampsImu[seq][first_imu[seq]] <=
                       vTimestampsCam[seq][ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                        vAcc[seq][first_imu[seq]].x,
                        vAcc[seq][first_imu[seq]].y,
                        vAcc[seq][first_imu[seq]].z,
                        vGyro[seq][first_imu[seq]].x,
                        vGyro[seq][first_imu[seq]].y,
                        vGyro[seq][first_imu[seq]].z,
                        vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++;
                }
            }

            std::chrono::steady_clock::time_point t1 =
                std::chrono::steady_clock::now();

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
            SLAM.TrackMonocular(im,
                                tframe,
                                vImuMeas);  // TODO change to monocular_inertial
            // SLAM.TrackMonocular(im, tframe, {}); // TODO change to
            // monocular_inertial

            static size_t frameIdx = 0;
            cout << "Frame " << frameIdx++ << " has been handled." << endl;

            std::chrono::steady_clock::time_point t2 =
                std::chrono::steady_clock::now();

            double ttrack =
                std::chrono::duration_cast<std::chrono::duration<double>>(t2 -
                                                                          t1)
                    .count();
            ttrack_tot += ttrack;
            // std::cout << "ttrack: " << ttrack << std::endl;

            vTimesTrack[ni] = ttrack;

            // Wait to load the next frame
            double T = 0;
            if (ni < nImages[seq] - 1)
                T = vTimestampsCam[seq][ni + 1] - tframe;
            else if (ni > 0)
                T = tframe - vTimestampsCam[seq][ni - 1];

            if (ttrack < T)
            {
                // usleep((T - ttrack) * 1e6); // 1e6
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(uint32_t((T - ttrack) * 1e6)));
            }
        }
        if (seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}

void LoadImages(const string&   strImagePath,
                const string&   strPathTimes,
                vector<string>& vstrImages,
                vector<double>& vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t / 1e9);
        }
    }
}

void LoadIMU(const string&        strImuPath,
             vector<double>&      vTimeStamps,
             vector<cv::Point3f>& vAcc,
             vector<cv::Point3f>& vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while (!fImu.eof())
    {
        string s;
        getline(fImu, s);
        if (s[0] == '#') continue;

        if (!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int    count = 0;
            while ((pos = s.find(',')) != string::npos)
            {
                item          = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item    = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0] / 1e9);
            vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
            vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
        }
    }
}

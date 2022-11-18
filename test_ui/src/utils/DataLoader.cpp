#include "DataLoader.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>

#include "utils/Log.h"

EuRoCDataLoader::EuRoCDataLoader(const std::string& folder)
    : m_folder_name(folder)
{
    APP_INFO("EuRoC Data starts to be loaded.");
    if (m_folder_name.back() == '/')
    {
        m_folder_name.pop_back();
    }
    m_folder_name += "/mav0/";

    {
        std::ifstream file(m_folder_name + "cam0/data.csv");
        assert(file.is_open() && "Image data.csv is not open.");

        std::string line;
        std::getline(file, line);

        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string        word;

            std::getline(iss, word, ',');
            size_t time_stamp = std::stoull(word);

            m_image_name_list.push_back(time_stamp);
        }
    }

    {
        std::ifstream file(m_folder_name + "imu0/data.csv");
        assert(file.is_open() && "Imu data.csv is not open.");

        std::string line;
        std::getline(file, line);

        std::vector<ImuData> imu_data_list;

        while (std::getline(file, line))
        {
            ImuData imu_data;

            std::istringstream iss(line);
            std::string        word;

            std::getline(iss, word, ',');
            imu_data.time_stamp = std::stoull(word);
            std::getline(iss, word, ',');
            imu_data.w_x = std::stod(word);
            std::getline(iss, word, ',');
            imu_data.w_y = std::stod(word);
            std::getline(iss, word, ',');
            imu_data.w_z = std::stod(word);
            std::getline(iss, word, ',');
            imu_data.a_x = std::stod(word);
            std::getline(iss, word, ',');
            imu_data.a_y = std::stod(word);
            std::getline(iss, word, ',');
            imu_data.a_z = std::stod(word);

            imu_data_list.push_back(std::move(imu_data));
        }


        // organize the list
        auto it = imu_data_list.begin();
        for (size_t time_stamp : m_image_name_list)
        {
            std::vector<ImuData> imu_part_list;
            while (it != imu_data_list.end() && it->time_stamp <= time_stamp)
            {
                imu_part_list.push_back(*it++);
            }
            m_imu_data_list.push_back(std::move(imu_part_list));
        }
    }

    APP_INFO("Data has been loaded.");
}

std::pair<std::vector<ImgData>, std::vector<ImuData>>
EuRoCDataLoader::getNextData() const
{
    std::pair<std::vector<ImgData>, std::vector<ImuData>> res;

    size_t      time_stamp = m_image_name_list[m_current_idx];
    std::string path =
        m_folder_name + "cam0/data/" + std::to_string(time_stamp) + ".png";
    cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
    APP_ASSERT(image.data != nullptr, "Load invalid image.");

    res.first.push_back({ time_stamp, image });
    res.second = m_imu_data_list[m_current_idx];

    ++m_current_idx;

    return res;
}
#pragma once
#include <vector>
#include <string>

#include "utils/DataType.h"

class EuRoCDataLoader
{
public:
	EuRoCDataLoader(const std::string& folder);
	EuRoCDataLoader(const EuRoCDataLoader&) = delete;
	EuRoCDataLoader& operator=(const EuRoCDataLoader&) = delete;

	bool hasNextData() const { return m_current_idx < m_image_name_list.size(); }
	std::pair<std::vector<ImgData>, std::vector<ImuData>> getNextData() const;

protected:
	std::string m_folder_name;
	std::vector<size_t> m_image_name_list;
	std::vector<std::vector<ImuData>> m_imu_data_list;
	mutable size_t m_current_idx = 0;
	size_t m_max_idx = 0;
};
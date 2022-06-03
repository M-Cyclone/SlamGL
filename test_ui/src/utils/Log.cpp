#include "Log.h"

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

std::vector<spdlog::sink_ptr> Log::s_log_sinks;
std::shared_ptr<spdlog::logger> Log::s_debug_logger;
std::shared_ptr<spdlog::logger> Log::s_app_logger;

void Log::init()
{
	s_log_sinks.emplace_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
	s_log_sinks.emplace_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>("slam_app.log", true));

	s_log_sinks[0]->set_pattern("%^[%T] %n: %v%$");
	s_log_sinks[1]->set_pattern("[%T] [%l] %n: %v");

	s_debug_logger = std::make_shared<spdlog::logger>("Debug", s_log_sinks.begin(), s_log_sinks.end());
	spdlog::register_logger(s_debug_logger);
	s_debug_logger->set_level(spdlog::level::trace);
	s_debug_logger->flush_on(spdlog::level::trace);

	s_app_logger = std::make_shared<spdlog::logger>("App", s_log_sinks.begin(), s_log_sinks.end());
	spdlog::register_logger(s_app_logger);
	s_app_logger->set_level(spdlog::level::trace);
	s_app_logger->flush_on(spdlog::level::trace);
}

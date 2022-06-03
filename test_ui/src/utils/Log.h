#pragma once
#include <memory>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

class Log
{
public:
	static void init();

	static auto getDebugLogger() { return s_debug_logger; }
	static auto getAppLogger() { return s_app_logger; }

private:
	static std::vector<spdlog::sink_ptr> s_log_sinks;
	static std::shared_ptr<spdlog::logger> s_debug_logger;
	static std::shared_ptr<spdlog::logger> s_app_logger;
};

#ifndef NDEBUG
	#define DEBUG_TRACE(...)		::Log::getDebugLogger()->trace(__VA_ARGS__)
	#define DEBUG_INFO(...)			::Log::getDebugLogger()->info(__VA_ARGS__)
	#define DEBUG_WARN(...)			::Log::getDebugLogger()->warn(__VA_ARGS__)
	#define DEBUG_ERROR(...)		::Log::getDebugLogger()->error(__VA_ARGS__)
	#define DEBUG_CRITICAL(...)		::Log::getDebugLogger()->critical(__VA_ARGS__)
#else
	#define DEBUG_TRACE(...)
	#define DEBUG_INFO(...)
	#define DEBUG_WARN(...)
	#define DEBUG_ERROR(...)
	#define DEBUG_CRITICAL(...)
#endif

#define APP_TRACE(...)			::Log::getAppLogger()->trace(__VA_ARGS__)
#define APP_INFO(...)			::Log::getAppLogger()->info(__VA_ARGS__)
#define APP_WARN(...)			::Log::getAppLogger()->warn(__VA_ARGS__)
#define APP_ERROR(...)			::Log::getAppLogger()->error(__VA_ARGS__)
#define APP_CRITICAL(...)		::Log::getAppLogger()->critical(__VA_ARGS__)

#define APP_ASSERT(check, ...)	if(!(check)) { ::Log::getAppLogger()->critical(__VA_ARGS__); throw -1; }
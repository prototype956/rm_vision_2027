/**
 * @file logger.cpp
 * @brief Logger::Init() 的具体实现
 *
 */
#include "logger.hpp"

#include <stdexcept>

#include <filesystem>
#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace mv {

void Logger::Init(const std::string& log_dir, spdlog::level::level_enum level, bool console_on) {
  std::lock_guard<std::mutex> lock(init_mutex_);
  if (initialized_) {
    return;  // 幂等：多次调用 Init 只有第一次生效，避免重复创建 sink
  }

  // 目录不存在时自动创建，而不是在外部预先创建——
  // 这样调用方只需要给路径字符串，不需要关心目录是否已存在
  std::filesystem::create_directories(log_dir);

  // 用时间戳命名日志文件，而不是用固定名字（如 app.log）：
  // 时间戳命名保留所有历史记录，方便复盘问题
  auto file_name =
      fmt::format("{}/{:%Y-%m-%d_%H-%M-%S}.log", log_dir, std::chrono::system_clock::now());

  std::vector<spdlog::sink_ptr> sinks;

  // 文件 sink 始终记录 trace 及以上，不受 level 参数影响：
  // 控制台只显示用户关心的级别（可能是 info），但文件要保留完整记录供调试
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
  file_sink->set_level(spdlog::level::trace);
  sinks.emplace_back(file_sink);

  // 控制台 sink 独立控制级别，允许部署时关闭（无 tty 的嵌入式环境）
  if (console_on) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(level);
    sinks.emplace_back(console_sink);
  }

  // logger 级别设为 trace（不在 logger 层过滤）：
  // 过滤由各 sink 自己决定，logger 层过滤会导致文件 sink 也被截断
  logger_ = std::make_shared<spdlog::logger>("mv", sinks.begin(), sinks.end());
  logger_->set_level(spdlog::level::trace);
  logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

  // warn 及以上立即 flush，确保程序崩溃前最后几条错误日志不丢失
  logger_->flush_on(spdlog::level::warn);

  // 注册为 spdlog 全局 logger：让使用了 spdlog 默认 API 的第三方库也能找到这个 logger
  spdlog::register_logger(logger_);

  initialized_ = true;
}

void Logger::SetLevel(spdlog::level::level_enum level) {
  if (logger_) {
    logger_->set_level(level);
  }
}

}  // namespace mv

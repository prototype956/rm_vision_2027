#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <filesystem>
#include <fmt/core.h>
#include <spdlog/spdlog.h>

namespace mv {

class Logger {
 public:
  static Logger& Instance() {
    static Logger instance;
    return instance;
  }

  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;
  Logger(Logger&&) = delete;
  Logger& operator=(Logger&&) = delete;

  void InitFromFile(const std::filesystem::path& config_path);

  template <typename... Args>
  void Info(const std::string& module, fmt::format_string<Args...> format, Args&&... args) {
    Log(spdlog::level::info, module, format, std::forward<Args>(args)...);
  }

  template <typename... Args>
  void Warn(const std::string& module, fmt::format_string<Args...> format, Args&&... args) {
    Log(spdlog::level::warn, module, format, std::forward<Args>(args)...);
  }

  template <typename... Args>
  void Error(const std::string& module, fmt::format_string<Args...> format, Args&&... args) {
    Log(spdlog::level::err, module, format, std::forward<Args>(args)...);
  }

 private:
  Logger() = default;

  void Init(const std::filesystem::path& log_dir, spdlog::level::level_enum console_level,
            bool console_on);

  template <typename... Args>
  void Log(spdlog::level::level_enum level, const std::string& module,
           fmt::format_string<Args...> format, Args&&... args) {
    if (!logger_) {
      Init("logs", spdlog::level::info, true);
    }
    logger_->log(level, "[{}] {}", module, fmt::format(format, std::forward<Args>(args)...));
  }

  std::shared_ptr<spdlog::logger> logger_;
  std::mutex init_mutex_;
};

}  // namespace mv

#define MV_LOG_INFO(module, ...) \
  mv::Logger::Instance().Info(module, __VA_ARGS__)  // NOLINT(cppcoreguidelines-macro-usage)
#define MV_LOG_WARN(module, ...) \
  mv::Logger::Instance().Warn(module, __VA_ARGS__)  // NOLINT(cppcoreguidelines-macro-usage)
#define MV_LOG_ERROR(module, ...) \
  mv::Logger::Instance().Error(module, __VA_ARGS__)  // NOLINT(cppcoreguidelines-macro-usage)

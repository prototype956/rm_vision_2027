#include "logger.hpp"

#include "config.hpp"

#include <chrono>
#include <stdexcept>
#include <vector>

#include <filesystem>
#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace mv {
namespace {

spdlog::level::level_enum ParseLogLevel(const std::string& value) {
  if (value == "trace")
    return spdlog::level::trace;
  if (value == "debug")
    return spdlog::level::debug;
  if (value == "info")
    return spdlog::level::info;
  if (value == "warn")
    return spdlog::level::warn;
  if (value == "error")
    return spdlog::level::err;
  if (value == "critical")
    return spdlog::level::critical;
  throw ConfigError("logger.level has unsupported value '" + value + "'");
}

}  // namespace

void Logger::InitFromFile(const std::filesystem::path& config_path) {
  const auto root = ConfigLoader::LoadFile(config_path);
  ConfigLoader::RejectUnknownKeys(root, {"schema_version", "log_dir", "level", "console"},
                                  "logger config");

  const auto raw_log_dir = ConfigLoader::Require<std::string>(root, "log_dir", "logger config");
  const auto level = ConfigLoader::Require<std::string>(root, "level", "logger config");
  const bool console = ConfigLoader::Require<bool>(root, "console", "logger config");
  if (raw_log_dir.empty()) {
    throw ConfigError("logger.log_dir must not be empty");
  }

  const auto log_dir = ConfigLoader::ResolvePath(config_path.parent_path(), raw_log_dir);
  Init(log_dir, ParseLogLevel(level), console);
  Info("Config", "logger config: {}",
       std::filesystem::absolute(config_path).lexically_normal().string());
}

void Logger::Init(const std::filesystem::path& log_dir, spdlog::level::level_enum console_level,
                  bool console_on) {
  std::lock_guard<std::mutex> lock(init_mutex_);
  if (logger_) {
    return;
  }

  std::filesystem::create_directories(log_dir);
  const auto file_path =
      log_dir / fmt::format("{:%Y-%m-%d_%H-%M-%S}.log", std::chrono::system_clock::now());

  std::vector<spdlog::sink_ptr> sinks;
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_path.string(), true);
  file_sink->set_level(spdlog::level::trace);
  sinks.emplace_back(std::move(file_sink));

  if (console_on) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(console_level);
    sinks.emplace_back(std::move(console_sink));
  }

  logger_ = std::make_shared<spdlog::logger>("mv", sinks.begin(), sinks.end());
  logger_->set_level(spdlog::level::trace);
  logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
  logger_->flush_on(spdlog::level::warn);
  spdlog::register_logger(logger_);
}

}  // namespace mv

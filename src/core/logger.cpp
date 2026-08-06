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
  const auto ROOT = ConfigLoader::LoadFile(config_path);
  ConfigLoader::RejectUnknownKeys(ROOT, {"schema_version", "log_dir", "level", "console"},
                                  "logger config");

  const auto RAW_LOG_DIR = ConfigLoader::Require<std::string>(ROOT, "log_dir", "logger config");
  const auto LEVEL = ConfigLoader::Require<std::string>(ROOT, "level", "logger config");
  const bool CONSOLE = ConfigLoader::Require<bool>(ROOT, "console", "logger config");
  if (RAW_LOG_DIR.empty()) {
    throw ConfigError("logger.log_dir must not be empty");
  }

  const auto LOG_DIR = ConfigLoader::ResolvePath(config_path.parent_path(), RAW_LOG_DIR);
  Init(LOG_DIR, ParseLogLevel(LEVEL), CONSOLE);
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
  const auto FILE_PATH =
      log_dir / fmt::format("{:%Y-%m-%d_%H-%M-%S}.log", std::chrono::system_clock::now());

  std::vector<spdlog::sink_ptr> sinks;
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(FILE_PATH.string(), true);
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

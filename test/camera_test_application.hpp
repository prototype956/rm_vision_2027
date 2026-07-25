#pragma once

#include "hal/camera/i_camera.hpp"

#include <filesystem>
#include <memory>

#include <yaml-cpp/yaml.h>

namespace mv::test {

struct CameraTestSettings {
  int duration_sec{0};
  int warmup_sec{0};
  bool preview{false};
  int report_interval_sec{0};
  int save_sample_interval_sec{0};
  int restart_cycles{0};
  int frames_per_restart_cycle{100};
  std::filesystem::path output_dir;
};

class CameraTestApplication final {
 public:
  CameraTestApplication(std::unique_ptr<hal::ICamera> camera, YAML::Node camera_config,
                        CameraTestSettings settings);
  ~CameraTestApplication();

  int Run();

 private:
  std::unique_ptr<hal::ICamera> camera_;
  YAML::Node camera_config_;
  CameraTestSettings settings_;
};

}  // namespace mv::test

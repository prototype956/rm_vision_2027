#include "app/main.hpp"

#include "core/config.hpp"
#include "core/logger.hpp"
#include "hal/camera/camera_factory.hpp"
#include "hal/gimbal/talos/talos_gimbal_command_sink.hpp"
#include "modules/armor_corner_refiner/armor_corner_refiner.hpp"
#include "modules/armor_detector/armor_detector_config.hpp"
#include "modules/armor_light_detector/armor_light_detector_config.hpp"
#include "modules/armor_pnp/armor_pnp_config.hpp"
#include "modules/armor_predictor/armor_predictor_config.hpp"
#include "modules/fire_control/fire_control_config.hpp"
#include "modules/gimbal_trajectory_planner/gimbal_trajectory_planner_config.hpp"
#include "runtime/control_runtime.hpp"
#include "runtime/runtime_diagnostics_sink.hpp"
#include "runtime/runtime_error_policy.hpp"
#include "runtime/runtime_supervisor.hpp"
#include "runtime/vision_pipeline.hpp"
#include "runtime/vision_runtime.hpp"
#include "runtime/vision_tuning.hpp"
#include "tool/debug/debug_window.hpp"
#include "tool/foxglove/foxglove_config.hpp"
#include "tool/foxglove/vision_debug_publisher.hpp"
#include "tool/simulation_evaluation/simulation_evaluation_config.hpp"
#include "tool/simulation_evaluation/simulation_evaluator.hpp"
#include "tool/web/web_debug_config.hpp"
#include "tool/web/web_debug_server.hpp"

#include <csignal>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <filesystem>

namespace mv::app {
namespace {

constexpr char K_WINDOW_NAME[] = "MiracleVision Camera Preview";
volatile std::sig_atomic_t g_stop_requested = 0;

void HandleStopSignal(int) noexcept {
  g_stop_requested = 1;
}

bool LoadDebugWindowEnabled(const std::filesystem::path& config_path) {
  constexpr char CONTEXT[] = "debug window config";
  const auto ROOT = ConfigLoader::LoadFile(config_path);
  ConfigLoader::RejectUnknownKeys(ROOT, {"schema_version", "enabled"}, CONTEXT);
  if (ConfigLoader::Require<int>(ROOT, "schema_version", CONTEXT) != 1)
    throw ConfigError("debug window config schema_version must be 1");
  return ConfigLoader::Require<bool>(ROOT, "enabled", CONTEXT);
}

/** @brief 应用层把传输无关运行时诊断适配到 Foxglove 发布器。 */
class FoxgloveDiagnosticsSink final : public runtime::IRuntimeDiagnosticsSink {
 public:
  explicit FoxgloveDiagnosticsSink(tool::foxglove::VisionDebugPublisher& publisher) noexcept
      : publisher_(publisher) {}

  void PublishVision(const frame::FramePacket& packet, const runtime::VisionFrameOutput& output,
                     const runtime::VisionFrameDiagnostics& diagnostics,
                     const std::optional<tool::simulation_evaluation::SimulationEvaluationResult>&
                         evaluation) noexcept override {
    publisher_.Publish(packet, output, diagnostics, evaluation);
  }

  void PublishControl(const runtime::ControlCycleOutput& output,
                      const runtime::ControlCycleDiagnostics& diagnostics) noexcept override {
    publisher_.PublishControl(output, diagnostics);
  }

  [[nodiscard]] runtime::RuntimeDiagnosticsHealth SnapshotHealth() const noexcept override {
    const auto STATS = publisher_.SnapshotStats();
    return {.available = publisher_.IsRunning(),
            .error_count = STATS.encoding_errors + STATS.live_errors + STATS.recording_errors};
  }

 private:
  tool::foxglove::VisionDebugPublisher& publisher_;
};

/** @brief 将同一运行时诊断分发到全部已启动的调试后端。 */
class DiagnosticsFanoutSink final : public runtime::IRuntimeDiagnosticsSink {
 public:
  void Add(runtime::IRuntimeDiagnosticsSink& sink) { sinks_.push_back(&sink); }
  [[nodiscard]] bool Empty() const noexcept { return sinks_.empty(); }

  void PublishVision(const frame::FramePacket& packet, const runtime::VisionFrameOutput& output,
                     const runtime::VisionFrameDiagnostics& diagnostics,
                     const std::optional<tool::simulation_evaluation::SimulationEvaluationResult>&
                         evaluation) noexcept override {
    for (auto* sink : sinks_)
      sink->PublishVision(packet, output, diagnostics, evaluation);
  }

  void PublishControl(const runtime::ControlCycleOutput& output,
                      const runtime::ControlCycleDiagnostics& diagnostics) noexcept override {
    for (auto* sink : sinks_)
      sink->PublishControl(output, diagnostics);
  }

  [[nodiscard]] runtime::RuntimeDiagnosticsHealth SnapshotHealth() const noexcept override {
    runtime::RuntimeDiagnosticsHealth combined{.available = false};
    for (const auto* sink : sinks_) {
      const auto HEALTH = sink->SnapshotHealth();
      combined.available = combined.available || HEALTH.available;
      combined.error_count += HEALTH.error_count;
    }
    return combined;
  }

 private:
  std::vector<runtime::IRuntimeDiagnosticsSink*> sinks_;
};

runtime::VisionFrontendTuningConfig MakeFrontendTuningConfig(
    const runtime::VisionPipelineConfig& config) {
  return {.detector = {.enemy_color = config.detector.enemy_color,
                       .confidence_threshold = config.detector.confidence_threshold,
                       .nms_iou_threshold = config.detector.nms_iou_threshold},
          .corner_refiner = config.corner_refiner,
          .light_detector = config.light_detector};
}

struct CameraSelection {
  std::string backend;                ///< 传给相机工厂的后端名称。
  std::filesystem::path config_path;  ///< 相对于配置根目录解析后的后端配置路径。
};

/** 读取主程序的相机选择，并在创建后端前完成名称和配置键校验。 */
CameraSelection LoadCameraSelection(const std::filesystem::path& config_root) {
  constexpr char CONTEXT[] = "main app config";
  const auto ROOT = ConfigLoader::LoadFile(config_root / "app/main.yaml");
  ConfigLoader::RejectUnknownKeys(ROOT, {"schema_version", "camera"}, CONTEXT);

  const auto CAMERA = ROOT["camera"];
  ConfigLoader::RequireMap(CAMERA, "main app config.camera");
  ConfigLoader::RejectUnknownKeys(CAMERA, {"backend", "configs"}, "main app config.camera");
  const auto BACKEND =
      ConfigLoader::Require<std::string>(CAMERA, "backend", "main app config.camera");

  const auto CONFIGS = CAMERA["configs"];
  ConfigLoader::RequireMap(CONFIGS, "main app config.camera.configs");
  ConfigLoader::RejectUnknownKeys(CONFIGS, {"mindvision", "talos"},
                                  "main app config.camera.configs");
  if (BACKEND != "mindvision" && BACKEND != "talos")
    throw ConfigError("main app config.camera.backend must be mindvision or talos");
  const auto CONFIG_FILE =
      ConfigLoader::Require<std::string>(CONFIGS, BACKEND, "main app config.camera.configs");
  return CameraSelection{BACKEND, ConfigLoader::ResolvePath(config_root, CONFIG_FILE)};
}

}  // namespace

int Run() {
  try {
    const std::filesystem::path CONFIG_ROOT = CONFIG_FILE_PATH;
    const std::filesystem::path PROJECT_ROOT = PROJECT_ROOT_PATH;
    Logger::Instance().InitFromFile(CONFIG_ROOT / "core/logger.yaml");
    std::signal(SIGINT, HandleStopSignal);
    std::signal(SIGTERM, HandleStopSignal);

    const auto ERROR_POLICY_YAML =
        ConfigLoader::LoadFile(CONFIG_ROOT / "runtime/error_policy.yaml", 1);
    runtime::RuntimeSupervisor supervisor(runtime::ParseRuntimeErrorPolicy(ERROR_POLICY_YAML));

    runtime::VisionPipelineConfig pipeline_config;
    try {
      const auto DETECTOR_YAML =
          ConfigLoader::LoadFile(CONFIG_ROOT / "modules/armor_detector.yaml");
      pipeline_config.detector = modules::ParseArmorDetectorConfig(DETECTOR_YAML, PROJECT_ROOT);
    } catch (const std::exception& error) {
      MV_LOG_ERROR("App", "armor detector initialization failed: {}", error.what());
      return 2;
    } catch (...) {
      MV_LOG_ERROR("App", "armor detector initialization failed: unknown exception");
      return 2;
    }

    const auto PNP_YAML = ConfigLoader::LoadFile(CONFIG_ROOT / "modules/armor_pnp.yaml", 3);
    pipeline_config.pnp = modules::ParseArmorPnpConfig(PNP_YAML);
    const auto PREDICTOR_YAML =
        ConfigLoader::LoadFile(CONFIG_ROOT / "modules/armor_predictor.yaml",
                               modules::ARMOR_PREDICTOR_CONFIG_SCHEMA_VERSION);
    pipeline_config.predictor = modules::ParseArmorPredictorConfig(PREDICTOR_YAML);
    const auto REFINER_YAML =
        ConfigLoader::LoadFile(CONFIG_ROOT / "modules/armor_corner_refiner.yaml");
    pipeline_config.corner_refiner = modules::ParseArmorCornerRefinerConfig(REFINER_YAML);
    const auto LIGHT_DETECTOR_YAML =
        ConfigLoader::LoadFile(CONFIG_ROOT / "modules/armor_light_detector.yaml",
                               modules::ARMOR_LIGHT_DETECTOR_CONFIG_SCHEMA_VERSION);
    pipeline_config.light_detector = modules::ParseArmorLightDetectorConfig(LIGHT_DETECTOR_YAML);
    runtime::VisionTuningMailbox tuning_mailbox(MakeFrontendTuningConfig(pipeline_config));

    std::unique_ptr<runtime::VisionPipeline> pipeline;
    try {
      pipeline = std::make_unique<runtime::VisionPipeline>(pipeline_config);
    } catch (const std::exception& error) {
      MV_LOG_ERROR("App", "armor detector initialization failed: {}", error.what());
      return 2;
    } catch (...) {
      MV_LOG_ERROR("App", "armor detector initialization failed: unknown exception");
      return 2;
    }

    std::unique_ptr<tool::simulation_evaluation::SimulationEvaluator> simulation_evaluator;
    try {
      const auto EVALUATION_YAML =
          ConfigLoader::LoadFile(CONFIG_ROOT / "tool/simulation_evaluation.yaml", 1);
      simulation_evaluator = std::make_unique<tool::simulation_evaluation::SimulationEvaluator>(
          tool::simulation_evaluation::ParseSimulationEvaluationConfig(EVALUATION_YAML),
          pipeline_config.pnp);
    } catch (const std::exception& error) {
      static_cast<void>(
          supervisor.Report(runtime::RuntimeFaultCode::EVALUATION_INIT_FAILURE, error.what()));
      MV_LOG_WARN("App", "simulation evaluation disabled after initialization failure: {}",
                  error.what());
    } catch (...) {
      static_cast<void>(supervisor.Report(runtime::RuntimeFaultCode::EVALUATION_INIT_FAILURE,
                                          "unknown evaluation initialization exception"));
      MV_LOG_WARN("App", "simulation evaluation disabled after unknown initialization failure");
    }

    const auto CAMERA_SELECTION = LoadCameraSelection(CONFIG_ROOT);
    const auto CAMERA_CONFIG = ConfigLoader::LoadFile(CAMERA_SELECTION.config_path);
    auto camera = hal::CreateCamera(CAMERA_SELECTION.backend);
    MV_LOG_INFO("Config", "camera backend={} config={}", CAMERA_SELECTION.backend,
                CAMERA_SELECTION.config_path.string());
    bool camera_opened = false;
    try {
      camera_opened = camera->Open(CAMERA_CONFIG);
    } catch (const std::exception& error) {
      MV_LOG_ERROR("App", "{} camera open failed: {}", CAMERA_SELECTION.backend, error.what());
      return 3;
    } catch (...) {
      MV_LOG_ERROR("App", "{} camera open failed: unknown exception", CAMERA_SELECTION.backend);
      return 3;
    }
    if (!camera_opened) {
      MV_LOG_ERROR("App", "{} camera open failed", CAMERA_SELECTION.backend);
      return 3;
    }

    const bool PREVIEW_ENABLED = LoadDebugWindowEnabled(CONFIG_ROOT / "tool/debug_window.yaml");
    std::unique_ptr<tool::DebugWindow> window;
    if (PREVIEW_ENABLED) {
      try {
        window = std::make_unique<tool::DebugWindow>(K_WINDOW_NAME);
      } catch (const std::exception& error) {
        static_cast<void>(
            supervisor.Report(runtime::RuntimeFaultCode::DEBUG_WINDOW_INIT_FAILURE, error.what()));
      } catch (...) {
        static_cast<void>(supervisor.Report(runtime::RuntimeFaultCode::DEBUG_WINDOW_INIT_FAILURE,
                                            "unknown debug window initialization exception"));
      }
    }

    std::unique_ptr<tool::foxglove::VisionDebugPublisher> foxglove_publisher;
    try {
      const auto FOXGLOVE_PATH = CONFIG_ROOT / "tool/foxglove.yaml";
      const auto FOXGLOVE_YAML = ConfigLoader::LoadFile(FOXGLOVE_PATH);
      auto foxglove_config = tool::foxglove::ParseConfig(FOXGLOVE_YAML, FOXGLOVE_PATH);
      if (foxglove_config.enabled) {
        foxglove_publisher =
            std::make_unique<tool::foxglove::VisionDebugPublisher>(std::move(foxglove_config));
        if (!foxglove_publisher->IsRunning()) {
          static_cast<void>(
              supervisor.Report(runtime::RuntimeFaultCode::DIAGNOSTICS_INIT_FAILURE,
                                "Foxglove configured but no live or recording sink started"));
          MV_LOG_WARN("App", "Foxglove configured but no live or recording sink started");
        }
      }
    } catch (const std::exception& error) {
      static_cast<void>(
          supervisor.Report(runtime::RuntimeFaultCode::DIAGNOSTICS_INIT_FAILURE, error.what()));
      MV_LOG_ERROR("App", "Foxglove disabled after initialization failure: {}", error.what());
      foxglove_publisher.reset();
    } catch (...) {
      static_cast<void>(supervisor.Report(runtime::RuntimeFaultCode::DIAGNOSTICS_INIT_FAILURE,
                                          "unknown Foxglove initialization exception"));
      MV_LOG_ERROR("App", "Foxglove disabled after unknown initialization failure");
      foxglove_publisher.reset();
    }

    std::unique_ptr<FoxgloveDiagnosticsSink> foxglove_diagnostics_sink;
    DiagnosticsFanoutSink diagnostics_fanout;
    if (foxglove_publisher) {
      foxglove_diagnostics_sink = std::make_unique<FoxgloveDiagnosticsSink>(*foxglove_publisher);
      diagnostics_fanout.Add(*foxglove_diagnostics_sink);
    }

    std::unique_ptr<tool::web::WebDebugServer> web_debug_server;
    try {
      const auto WEB_YAML = ConfigLoader::LoadFile(CONFIG_ROOT / "tool/web.yaml");
      auto web_config = tool::web::ParseConfig(WEB_YAML);
      if (web_config.enabled) {
        web_debug_server = std::make_unique<tool::web::WebDebugServer>(
            std::move(web_config), tuning_mailbox, pipeline_config, PROJECT_ROOT);
        if (web_debug_server->Start()) {
          diagnostics_fanout.Add(*web_debug_server);
        }
      }
    } catch (const std::exception& error) {
      MV_LOG_WARN("WebDebug", "disabled after initialization failure: {}", error.what());
      web_debug_server.reset();
    } catch (...) {
      MV_LOG_WARN("WebDebug", "disabled after unknown initialization failure");
      web_debug_server.reset();
    }
    runtime::IRuntimeDiagnosticsSink* diagnostics_sink =
        diagnostics_fanout.Empty() ? nullptr : &diagnostics_fanout;

    std::unique_ptr<runtime::ControlRuntime> control_runtime;
    if (CAMERA_SELECTION.backend == "talos") {
      const auto FIRE_YAML = ConfigLoader::LoadFile(CONFIG_ROOT / "modules/fire_control.yaml");
      const auto PLANNER_YAML =
          ConfigLoader::LoadFile(CONFIG_ROOT / "modules/gimbal_trajectory_planner.yaml");
      const auto FIRE_CONFIG = modules::ParseFireControlConfig(FIRE_YAML);
      const auto PLANNER_CONFIG = modules::ParseGimbalTrajectoryPlannerConfig(PLANNER_YAML);
      auto command_sink = std::make_unique<hal::TalosGimbalCommandSink>();
      if (!command_sink->Open(CAMERA_CONFIG)) {
        MV_LOG_ERROR("App", "Talos command sink initialization failed");
        return 6;
      }
      try {
        control_runtime = std::make_unique<runtime::ControlRuntime>(
            FIRE_CONFIG, PLANNER_CONFIG, std::move(command_sink), diagnostics_sink, supervisor);
        control_runtime->Start();
      } catch (const std::exception& error) {
        static_cast<void>(
            supervisor.Report(runtime::RuntimeFaultCode::CONTROL_THREAD_EXCEPTION, error.what()));
        MV_LOG_ERROR("App", "Talos control runtime initialization failed: {}", error.what());
        return 7;
      } catch (...) {
        static_cast<void>(supervisor.Report(runtime::RuntimeFaultCode::CONTROL_THREAD_EXCEPTION,
                                            "unknown control runtime initialization exception"));
        MV_LOG_ERROR("App", "Talos control runtime initialization failed: unknown exception");
        return 7;
      }
      MV_LOG_INFO("Control", "Talos 100 Hz trajectory planning and fire control started");
    }

    runtime::VisionRuntime vision_runtime(*camera, *pipeline, control_runtime.get(), window.get(),
                                          diagnostics_sink, simulation_evaluator.get(), supervisor,
                                          &tuning_mailbox);
    return runtime::RuntimeExitCode(
        vision_runtime.Run([] { return g_stop_requested != 0; }).reason);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[App] FATAL: %s\n", error.what());
    return 1;
  } catch (...) {
    std::fprintf(stderr, "[App] FATAL: unknown exception\n");
    return 1;
  }
}

}  // namespace mv::app

int main() {
  return mv::app::Run();
}

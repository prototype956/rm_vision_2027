#include "hal/serial/attitude_wire.h"
#include "hal/serial/serial_gimbal_command_sink.hpp"
#include "modules/fire_control/control_session.hpp"
#include "modules/fire_control/fire_only_policy_adapter.hpp"
#include "runtime/gimbal_tuning_reload.hpp"
#include "runtime/referee_observation_adapter.hpp"
#include "runtime/serial_gimbal_control_adapter.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

#include <filesystem>
#include <numbers>
using namespace mv;
using namespace mv::modules;
using Time = std::chrono::steady_clock::time_point;
static int checks = 0;
void Check(bool value, const char* message) {
  ++checks;
  if (!value)
    throw std::runtime_error(message);
}
Time At(int ms) {
  return Time(std::chrono::milliseconds(ms));
}
ControlInputSnapshot Sample(int ms) {
  ControlInputSnapshot s;
  auto& e = s.prediction;
  e.sequence = static_cast<std::uint64_t>(ms) + 1;
  e.source_steady_time = At(ms);
  e.source_round_id = 1;
  e.track_generation = 1;
  e.state = TrackerState::TRACKING;
  e.label = ArmorLabel::THREE;
  e.type = geometry::ArmorType::SMALL;
  e.center_world = {4, 0, .1};
  e.radii_m = {.21, .23};
  e.armor_tilt_rad = -.265;
  e.orientation_world = Eigen::AngleAxisd(std::numbers::pi, geometry::Vector3::UnitZ());
  s.external_control_enabled = true;
  s.referee.valid = true;
  s.referee.alive = true;
  s.referee.fire_permitted = true;
  s.referee.heat_limit = 88;
  s.referee.unlimited = true;
  s.referee.received_at = At(ms);
  return s;
}
hal::GimbalFeedback Feedback(int ms) {
  hal::GimbalFeedback f;
  f.valid = true;
  f.timestamp = At(ms);
  return f;
}

/** @brief 验证真实运行时适配器的速率/停止路径，不打开硬件设备。 */
class FakeRateChannel final : public hal::IGimbalRateCommandSink {
 public:
  bool healthy{true};
  bool send_succeeds{true};
  hal::GimbalActuatorTelemetry actual;
  hal::GimbalCommand target;
  hal::GimbalRateCommand rate;
  bool SendRate(const hal::GimbalCommand& next,
                const hal::GimbalRateCommand& next_rate) noexcept override {
    target = next;
    rate = next_rate;
    actual.command_valid = healthy && send_succeeds && next.valid && next_rate.valid;
    return healthy && send_succeeds;
  }
  bool IsHealthy() const noexcept override { return healthy; }
  bool ExternalControlEnabled() const noexcept override { return healthy; }
  std::uint64_t HeartbeatTimestampNs() const noexcept override { return 1; }
  hal::GimbalActuatorTelemetry ActuatorTelemetry() const noexcept override { return actual; }
};

int main(int argc, char** argv) {
  try {
    if (argc != 2)
      return 2;
    auto cfg = ParseFireControlConfig(YAML::LoadFile(std::string(argv[1]) + "/fire_control.yaml"));
    auto pcfg = LoadGimbalControlConfig(std::filesystem::path(argv[1]).parent_path(), "talos");
    cfg.auto_fire = true;
    auto yaml = YAML::LoadFile(std::string(argv[1]) + "/fire_control.yaml");
    yaml.remove("test_bypass_referee");
    Check(!ParseFireControlConfig(yaml).test_bypass_referee, "referee checks default on");
    yaml["test_bypass_referee"] = true;
    Check(ParseFireControlConfig(yaml).test_bypass_referee, "explicit referee bypass parsed");
    // Existing competition checks must not inherit the deployed bench-mode setting.
    cfg.test_bypass_referee = false;
    {
      auto config_yaml = YAML::LoadFile(std::string(argv[1]) + "/fire_control.yaml");
      config_yaml["timing"].remove("friction_hold_s");
      Check(ParseFireControlConfig(config_yaml).friction_hold_s == 1.0,
            "friction hold defaults to one second");
      for (double bad : {-0.1, 10.1, std::numeric_limits<double>::infinity()}) {
        config_yaml["timing"]["friction_hold_s"] = bad;
        bool rejected = false;
        try {
          static_cast<void>(ParseFireControlConfig(config_yaml));
        } catch (const std::exception&) {
          rejected = true;
        }
        Check(rejected, "invalid friction hold rejected");
      }
      for (int enabled = 0; enabled < 2; ++enabled) {
        for (int fire = 0; fire < 2; ++fire) {
          for (int friction = 0; friction < 2; ++friction) {
            const auto flags = AvGimbalCommandFlags(enabled, fire, friction);
            Check(((flags & AV_GIMBAL_FLAG_FIRE_REQUEST) != 0) == (enabled && fire),
                  "wire feeder bit independently gated");
            Check(
                ((flags & AV_GIMBAL_FLAG_FRICTION_REQUEST) != 0) == (enabled && (fire || friction)),
                "wire friction bit independently gated");
          }
        }
      }
      auto direct = pcfg;
      direct.mode = GimbalControlMode::PID;
      direct.mpc.reset();
      auto shooter_cfg = cfg;
      shooter_cfg.friction_hold_s = 1.0;
      ControlSession session(shooter_cfg, direct);
      const auto step = [&](int ms, TrackerState state, bool enabled = true, bool healthy = true,
                            bool sent = true, int action = 1, int source_ms = -1) {
        auto input = Sample(source_ms < 0 ? ms : source_ms);
        input.prediction.state = state;
        input.external_control_enabled = enabled;
        // Match the real serial sink: IMU feedback continues even without images.
        hal::GimbalActuatorTelemetry actuator;
        actuator.valid = true;
        actuator.mode = hal::GimbalActuatorMode::PHYSICAL;
        actuator.state_timestamp_ns = 1000000000ULL + ms * 1000000ULL;
        auto result = session.Step(input, actuator, healthy, At(ms), actuator.state_timestamp_ns,
                                   PolicyDecision{action});
        session.AcknowledgePublication(result, sent, At(ms));
        return result.output.command;
      };
      auto command = step(0, TrackerState::TEMP_LOST);
      Check(!command.friction_wheel && !command.fire, "no spin-up from prediction-only startup");
      command = step(10, TrackerState::DETECTING);
      Check(command.friction_wheel && !command.fire,
            "visible aim target spins friction before fire");
      command = step(20, TrackerState::TRACKING, true, true, true, 2);
      Check(command.friction_wheel && command.fire, "accepted shot requests friction and feeder");
      command = step(30, TrackerState::TEMP_LOST);
      Check(command.friction_wheel && !command.fire, "loss cancels feeder immediately");
      command = step(900, TrackerState::TEMP_LOST);
      Check(command.friction_wheel && !command.valid && !command.fire,
            "friction hold outlives motion extrapolation without feeding");
      command = step(1020, TrackerState::TEMP_LOST);
      Check(!command.friction_wheel && !command.fire, "friction expires at one second exactly");
      Check(step(1030, TrackerState::TRACKING).friction_wheel, "reacquisition spins up again");
      Check(!step(1040, TrackerState::TEMP_LOST, false).friction_wheel,
            "control disable immediately clears hold");
      Check(!step(1050, TrackerState::TEMP_LOST).friction_wheel,
            "reenabling while lost does not restore previous hold");
      Check(step(1060, TrackerState::TRACKING).friction_wheel, "new target restores spin-up");
      Check(!step(1070, TrackerState::TEMP_LOST, true, false).friction_wheel,
            "unhealthy link cancels hold");
      Check(!step(1080, TrackerState::TEMP_LOST).friction_wheel,
            "reconnection while lost cannot reuse hold");
      Check(!step(1090, TrackerState::TRACKING, true, true, false).friction_wheel,
            "send failure cancels friction request");
      Check(!step(1100, TrackerState::TEMP_LOST).friction_wheel,
            "send failure clears hold history");
      Check(step(1110, TrackerState::TRACKING).friction_wheel, "target after error spins up");
      session.Reset();
      Check(!step(1120, TrackerState::TEMP_LOST).friction_wheel, "session reset clears hold");
      Check(step(1140, TrackerState::TRACKING).friction_wheel, "camera recovery spins up");
      Check(step(1200, TrackerState::TRACKING, true, true, true, 1, 1140).friction_wheel,
            "short camera gap retains spin-up");
      Check(!step(2150, TrackerState::TRACKING, true, true, true, 1, 1140).friction_wheel,
            "reusing an old tracking snapshot cannot keep friction running forever");
    }
    {
      const auto root = std::filesystem::path(argv[1]).parent_path();
      auto pid_config = LoadGimbalControlConfig(root, "mindvision");
      Check(pid_config.mode == GimbalControlMode::PID && pid_config.pid && !pid_config.mpc,
            "real deployment loads PID without MPC parameters");
      Check(pcfg.mode == GimbalControlMode::MPC && pcfg.mpc && !pcfg.pid,
            "Talos deployment loads MPC without PID parameters");
      GimbalReferencePlanner pid_planner(pid_config);
      std::vector<AimReferencePoint> reference(static_cast<std::size_t>(pcfg.horizon_steps));
      for (auto& point : reference) {
        point.yaw = .03;
        point.pitch = .2;
      }
      auto plan = pid_planner.Plan(Feedback(0), reference);
      Check(plan.output.valid && plan.output.command.yaw == reference[1].yaw &&
                plan.output.command.pitch == reference[1].pitch &&
                plan.output.command.yaw_velocity == 0.,
            "PID selects the direct next reference");
      Check(plan.diagnostics.yaw_iterations == 0 && plan.diagnostics.pitch_iterations == 0 &&
                plan.diagnostics.warm_start_action == GimbalWarmStartAction::NONE,
            "PID has no solver or warm start");
      pid_planner.RequestWarmStartRebase();
      Check(pid_planner.Plan(Feedback(0), reference).output.valid,
            "PID target rebase stays direct");
      reference[1].pitch = std::numeric_limits<double>::quiet_NaN();
      Check(!pid_planner.Plan(Feedback(0), reference).output.valid,
            "PID rejects nonfinite targets");
      Check(!pid_planner.Plan(Feedback(0), {}).output.valid, "PID rejects empty references");
      auto common = YAML::LoadFile((root / "modules/gimbal_control.yaml").string());
      auto pid_yaml = YAML::LoadFile((root / "modules/gimbal_control/pid.yaml").string());
      auto bad = YAML::Clone(common);
      bad["modes"]["mindvision"] = "bang_bang";
      bool rejected = false;
      try {
        (void)ParseGimbalControlConfig(bad, "mindvision", pid_yaml);
      } catch (const std::exception&) {
        rejected = true;
      }
      Check(rejected, "removed experimental mode is rejected");
      bad = YAML::Clone(common);
      bad["test_pid_bypass"] = true;
      rejected = false;
      try {
        (void)ParseGimbalControlConfig(bad, "mindvision", pid_yaml);
      } catch (const std::exception&) {
        rejected = true;
      }
      Check(rejected, "legacy mode switches are rejected instead of silently falling back");
      auto invalid_pid = YAML::Clone(pid_yaml);
      invalid_pid["pitch"]["integral_stop_error_rad"] = .001;
      rejected = false;
      try {
        (void)ParsePidRateControllerConfig(invalid_pid);
      } catch (const std::exception&) {
        rejected = true;
      }
      Check(rejected, "invalid integral thresholds rejected");
      invalid_pid = YAML::Clone(pid_yaml);
      invalid_pid["yaw"]["ki"] = -1.;
      rejected = false;
      try {
        (void)ParsePidRateControllerConfig(invalid_pid);
      } catch (const std::exception&) {
        rejected = true;
      }
      Check(rejected, "negative PID gain rejected");

      const auto directory =
          std::filesystem::temp_directory_path() /
          ("mv-gimbal-reload-" +
           std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
      std::filesystem::create_directories(directory / "modules/gimbal_control");
      std::filesystem::create_directories(directory / "hal/serial");
      struct Cleanup {
        std::filesystem::path directory;
        ~Cleanup() {
          std::error_code ec;
          std::filesystem::remove_all(directory, ec);
        }
      } cleanup{directory};
      const auto save = [&](const std::filesystem::path& relative, const std::string& text) {
        const auto path = directory / relative;
        {
          std::ofstream output(path.string() + ".new");
          output << text;
        }
        std::filesystem::rename(path.string() + ".new", path);
      };
      auto transport_yaml = YAML::LoadFile((root / "hal/serial/gimbal.yaml").string());
      save("modules/gimbal_control.yaml", YAML::Dump(common));
      save("modules/gimbal_control/pid.yaml", YAML::Dump(pid_yaml));
      save("hal/serial/gimbal.yaml", YAML::Dump(transport_yaml));
      Check(LoadGimbalControlConfig(directory, "mindvision") == pid_config,
            "PID loads with no MPC file at all");
      runtime::GimbalTuningSnapshot initial{
          pid_config, hal::ParseSerialGimbalCommandSinkConfig(transport_yaml)};
      runtime::GimbalTuningReload reload(directory, initial);
      const auto wait_pending = [&]() {
        std::optional<runtime::GimbalTuningSnapshot> pending;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        do {
          pending = reload.TakePending();
          if (pending)
            break;
          std::this_thread::sleep_for(std::chrono::milliseconds(25));
        } while (std::chrono::steady_clock::now() < deadline);
        return pending;
      };
      auto next = YAML::Clone(pid_yaml);
      next["pitch"]["kp"] = initial.control.pid->pitch.kp + 1.;
      save("modules/gimbal_control/pid.yaml", YAML::Dump(next));
      auto pending = wait_pending();
      Check(pending && pending->control.pid->pitch.kp == initial.control.pid->pitch.kp + 1. &&
                pending->control.pid->yaw == initial.control.pid->yaw,
            "complete hot update preserves untouched axis");
      Check(!reload.TakePending(), "update is consumed once");
      save("modules/gimbal_control/pid.yaml", YAML::Dump(next) + "\n# comment\n");
      std::this_thread::sleep_for(std::chrono::milliseconds(750));
      Check(!reload.TakePending(), "comment save does not reset PID");
      save("modules/gimbal_control/pid.yaml", "pitch: [");
      std::this_thread::sleep_for(std::chrono::milliseconds(750));
      Check(!reload.TakePending(), "broken YAML preserves live parameters");
      save("modules/gimbal_control/pid.yaml", YAML::Dump(next));
      bad = YAML::Clone(common);
      bad["rate_limits"]["max_pitch_rate_rad_s"] = 5.;
      save("modules/gimbal_control.yaml", YAML::Dump(bad));
      std::this_thread::sleep_for(std::chrono::milliseconds(750));
      Check(!reload.TakePending(), "out-of-range rate rejected");
      bad = YAML::Clone(common);
      bad["reference"]["dt_s"] = .02;
      save("modules/gimbal_control.yaml", YAML::Dump(bad));
      std::this_thread::sleep_for(std::chrono::milliseconds(750));
      Check(!reload.TakePending(), "reference topology change requires restart");
      save("modules/gimbal_control.yaml", YAML::Dump(common));
      bad = YAML::Clone(transport_yaml);
      bad["fire_output_enabled"] = !initial.transport.fire_output_enabled;
      save("hal/serial/gimbal.yaml", YAML::Dump(bad));
      std::this_thread::sleep_for(std::chrono::milliseconds(750));
      Check(!reload.TakePending(), "transport enable changes require restart");
      save("hal/serial/gimbal.yaml", YAML::Dump(transport_yaml));
      save("modules/gimbal_control/pid.yaml", YAML::Dump(pid_yaml));
      pending = wait_pending();
      Check(pending && *pending == initial, "valid tuning recovers after rejection");
      auto mpc_yaml = YAML::LoadFile((root / "modules/gimbal_control/mpc.yaml").string());
      auto mpc_common = YAML::Clone(common);
      mpc_common["modes"]["mindvision"] = "mpc";
      save("modules/gimbal_control.yaml", YAML::Dump(mpc_common));
      save("modules/gimbal_control/mpc.yaml", YAML::Dump(mpc_yaml));
      std::filesystem::remove(directory / "modules/gimbal_control/pid.yaml");
      const auto mpc_config = LoadGimbalControlConfig(directory, "mindvision");
      Check(mpc_config.mpc && !mpc_config.pid, "MPC loads with no PID file");
      runtime::GimbalTuningReload mpc_reload(directory, {mpc_config, initial.transport});
      auto changed_mpc = YAML::Clone(mpc_yaml);
      changed_mpc["follower"]["yaw_kp"] = mpc_config.follower.yaw_kp + 1.;
      save("modules/gimbal_control/mpc.yaml", YAML::Dump(changed_mpc));
      std::this_thread::sleep_for(std::chrono::milliseconds(750));
      const auto FOLLOWER_UPDATE = mpc_reload.TakePending();
      Check(FOLLOWER_UPDATE &&
                FOLLOWER_UPDATE->control.follower.yaw_kp == mpc_config.follower.yaw_kp + 1.,
            "MPC follower hot updates independently");
      changed_mpc["planner"]["cost"]["q_yaw"][0] = 100.;
      save("modules/gimbal_control/mpc.yaml", YAML::Dump(changed_mpc));
      std::this_thread::sleep_for(std::chrono::milliseconds(750));
      Check(!mpc_reload.TakePending(), "MPC solver edits require restart");
    }
    {
      VariableIntegralRatePidConfig p;
      VariableIntegralRatePid pid;
      p.kp = 2.;
      p.ki = 1.;
      p.kd = .5;
      p.derivative_filter_tau_s = 0.;
      p.deadband_rad = 0.;
      p.integral_full_error_rad = .02;
      p.integral_stop_error_rad = .10;
      auto o = pid.Step(.01, .1, .01, 3., p);
      Check(std::abs(o.p - .02) < 1e-12 && std::abs(o.d + .05) < 1e-12 &&
                std::abs(o.i - .0001) < 1e-12 && o.integral_weight == 1.,
            "PID term units and measured-rate derivative sign");
      o = pid.Step(.06, 0., .01, 3., p);
      Check(std::abs(o.integral_weight - .5) < 1e-12 && std::abs(o.i - .0004) < 1e-12,
            "variable integral tapers linearly at intermediate error");
      o = pid.Step(.10, 0., .01, 3., p);
      Check(o.i == 0. && o.integral_weight == 0., "large error clears integral");
      p.integral_limit_rad_s = .001;
      for (int n = 0; n < 100; ++n)
        o = pid.Step(.01, 0., .01, 3., p);
      Check(std::abs(o.i - .001) < 1e-12, "integral contribution clamp");
      p.deadband_rad = .002;
      o = pid.Step(.001, .3, .01, 3., p);
      Check(o.rate_command == 0. && o.i == 0., "deadband outputs zero and clears I");
      p.kp = 100.;
      o = pid.Step(.02, 0., .01, .5, p);
      Check(o.rate_command == .5 && o.i == 0., "positive saturation blocks winding up");
      o = pid.Step(-.02, 0., .01, .5, p);
      Check(o.rate_command == -.5 && o.i == 0., "negative saturation blocks winding up");
      p.kp = 0.;
      static_cast<void>(pid.Step(.01, 0., .01, 3., p));
      o = pid.Step(.01, 0., .2, 3., p);
      Check(o.i == 0., "long scheduling gap resets integral instead of integrating gap");
      p.ki = 0.;
      p.derivative_filter_tau_s = .03;
      pid.Reset();
      static_cast<void>(pid.Step(.03, 0., .01, 3., p));
      o = pid.Step(.03, 1., .01, 3., p);
      Check(std::abs(o.d + .125) < 1e-12, "measured-rate D is low-pass filtered");
      pid.Reset();
      o = pid.Step(.08, 0., .01, 3., p);
      Check(o.d == 0., "reference step does not create derivative kick");
      o = pid.Step(std::numeric_limits<double>::quiet_NaN(), 0., .01, 3., p);
      Check(o.rate_command == 0., "nonfinite PID input produces zero");
    }
    {
      const auto CONFIG =
          LoadGimbalControlConfig(std::filesystem::path(argv[1]).parent_path(), "mindvision");
      auto channel = std::make_unique<FakeRateChannel>();
      auto* observed = channel.get();
      observed->actual.valid = true;
      observed->actual.actual_yaw = .10;
      observed->actual.actual_pitch = .20;
      runtime::SerialGimbalControlAdapter adapter(std::move(channel), {CONFIG, {}});
      hal::GimbalCommand target;
      target.valid = true;
      target.yaw = .11;
      target.pitch = .21;
      target.target_distance_m = 4.;
      Check(adapter.Send(target), "runtime adapter sends PID rates");
      Check(std::abs(observed->rate.yaw_rate_rad_s - .10) < 1e-12 &&
                std::abs(observed->rate.pitch_rate_rad_s - .085) < 1e-12,
            "PID uses direct send-stage feedback rather than projected feedback");
      Check(adapter.ActuatorTelemetry().serial_pid_active,
            "queued PID terms remain available through angle sink telemetry");
      target.valid = false;
      target.friction_wheel = true;
      Check(adapter.Send(target) && !observed->rate.valid && observed->target.friction_wheel,
            "invalid motion preserves friction-only intent with zero-rate transport");
      Check(!adapter.ActuatorTelemetry().serial_pid_active, "stop clears PID telemetry");
      target.valid = true;
      observed->send_succeeds = false;
      Check(!adapter.Send(target) && !adapter.ActuatorTelemetry().serial_pid_active,
            "send failure resets PID and reports failure");
      observed->send_succeeds = true;
      observed->healthy = false;
      Check(!adapter.Send(target) && !observed->rate.valid,
            "unhealthy feedback requests stopped transport");
      observed->healthy = true;
      Check(adapter.Send(target), "same adapter recovers without mode fallback");
      target.yaw = std::numeric_limits<double>::quiet_NaN();
      Check(!adapter.Send(target), "adapter rejects malformed aim before transport");

      PidRateControllerConfig tuning;
      tuning.yaw.ki = tuning.pitch.ki = 1.;
      tuning.yaw.kp = tuning.pitch.kp = 0.;
      hal::GimbalActuatorTelemetry actual;
      actual.valid = true;
      target.yaw = target.pitch = .01;
      PidRateController controller(tuning, {});
      const auto FIRST = controller.Step(target, actual, .01);
      tuning.pitch.kp = 2.;
      controller.ApplyTuning(tuning, {});
      const auto SECOND = controller.Step(target, actual, .01);
      Check(std::abs(SECOND.yaw.i - 2. * FIRST.yaw.i) < 1e-12 &&
                std::abs(SECOND.pitch.i - FIRST.pitch.i) < 1e-12,
            "changing pitch tuning resets pitch only and preserves yaw integral");
      auto limits = GimbalRateLimits{};
      limits.max_yaw_rate_rad_s = 2.;
      controller.ApplyTuning(tuning, limits);
      const auto THIRD = controller.Step(target, actual, .01);
      Check(std::abs(THIRD.yaw.i - FIRST.yaw.i) < 1e-12 &&
                std::abs(THIRD.pitch.i - 2. * FIRST.pitch.i) < 1e-12,
            "changing yaw rate limit resets yaw only");
      const auto MPC_RATE = FollowMpcRate(target, actual, pcfg.follower, pcfg.rate_limits);
      Check(MPC_RATE.valid && std::abs(MPC_RATE.yaw_rate_rad_s - .01) < 1e-12 &&
                std::abs(MPC_RATE.pitch_rate_rad_s - .07) < 1e-12,
            "MPC follower preserves deployed proportional and feed-forward law");
    }
    auto run = [&](FireControl& c, const ControlInputSnapshot& s, int ms, int action) {
      return c.Step(s, Feedback(ms), At(ms),
                    1000000000ULL + static_cast<std::uint64_t>(ms) * 1000000ULL,
                    PolicyDecision{action});
    };
    for (int action = 0; action <= 8; ++action) {
      FireControl c(cfg, pcfg);
      auto out = run(c, Sample(0), 0, action).output;
      Check(action == 0 ? !out.command.valid : out.selected_slot == (action - 1) / 2,
            "nine-action selection");
      if (action > 0)
        Check(out.command.valid, "nine-action MPC solve");
      Check(out.shot_accepted == (action > 0 && action % 2 == 0),
            "single-shot admission at time zero");
    }
    {
      FireControl c(cfg, pcfg);
      auto a = run(c, Sample(0), 0, 2);
      auto b = run(c, Sample(10), 10, 0);
      Check(a.output.shot_accepted && b.output.command.fire && !b.output.shot_requested &&
                !b.output.shot_accepted,
            "WAIT maintains one admitted pulse");
      auto busy = run(c, Sample(20), 20, 4);
      Check(busy.output.selected_slot == 1 && !busy.output.shot_accepted &&
                busy.output.reject_reason == FireRejectReason::PULSE_BUSY,
            "switch while pulse busy");
      auto cooldown = run(c, Sample(40), 40, 2);
      Check(!cooldown.output.command.fire &&
                cooldown.output.reject_reason == FireRejectReason::COOLDOWN,
            "interval rejects request");
      auto idle = run(c, Sample(100), 100, 0);
      Check(!idle.output.command.fire, "rejected request is never queued");
      auto switched = run(c, Sample(110), 110, 6);
      Check(switched.output.selected_slot == 2 && switched.output.shot_accepted,
            "switch and shoot same cycle");
    }
    {
      FireControl c(cfg, pcfg);
      static_cast<void>(run(c, Sample(0), 0, 2));
      auto s = Sample(10);
      s.prediction.track_generation = 2;
      auto o = run(c, s, 10, 2).output;
      Check(o.tracking_object_reset && !o.shot_accepted && !o.command.fire,
            "new target clears pulse but preserves interval");
      s = Sample(20);
      s.prediction.source_round_id = 2;
      o = run(c, s, 20, 2).output;
      Check(o.shot_accepted, "round reset clears mechanical history");
    }
    for (int mode = 0; mode < 7; ++mode) {
      FireControl c(cfg, pcfg);
      auto s = Sample(400);
      if (mode == 0)
        s.referee.valid = false;
      if (mode == 1)
        s.referee.received_at = At(0);
      if (mode == 2)
        s.referee.fire_permitted = false;
      if (mode == 3) {
        s.referee.unlimited = false;
        s.referee.allowance_remaining = 0;
      }
      if (mode == 4)
        s.referee.heat = std::numeric_limits<double>::quiet_NaN();
      if (mode == 5)
        s.referee.received_at = At(500);
      if (mode == 6)
        s.referee.alive = false;
      auto o = run(c, s, 400, 2).output;
      Check(o.command.valid && !o.command.fire && !o.shot_accepted,
            "referee failure tracks but blocks shot");
      Check(!c.Observe(s, Feedback(400), At(400)).action_mask[2],
            "referee failure masks firing in competition mode");
      auto bench_cfg = cfg;
      bench_cfg.test_bypass_referee = true;
      FireControl bench(bench_cfg, pcfg);
      Check(bench.Observe(s, Feedback(400), At(400)).action_mask[2],
            "bench mode unmasks referee-only failure");
      auto bench_out = run(bench, s, 400, 2).output;
      Check(bench_out.command.fire && bench_out.shot_accepted,
            "bench mode bypasses referee-only failure at final gate");
      Check(run(c, Sample(410), 410, 2).output.shot_accepted, "referee recovery");
    }
    for (int mode = 0; mode < 3; ++mode) {
      auto bench_cfg = cfg;
      bench_cfg.test_bypass_referee = true;
      if (mode == 0)
        bench_cfg.auto_fire = false;
      FireControl bench(bench_cfg, pcfg);
      auto s = Sample(400);
      s.referee.valid = false;
      if (mode == 1)
        s.external_control_enabled = false;
      if (mode == 2)
        s.prediction.state = TrackerState::TEMP_LOST;
      Check(!bench.Observe(s, Feedback(400), At(400)).action_mask[2],
            "bench mode retains non-referee action gates");
      Check(!run(bench, s, 400, 2).output.command.fire,
            "bench mode retains non-referee final gates");
    }
    {
      FireControl c(cfg, pcfg);
      static_cast<void>(run(c, Sample(0), 0, 1));
      auto s = Sample(10);
      s.prediction.state = TrackerState::TEMP_LOST;
      auto o = run(c, s, 10, 2).output;
      Check(o.command.valid && !o.command.fire && !o.tracking_object_reset,
            "temporary loss tracks without firing or identity reset");
      s = Sample(170);
      s.prediction.state = TrackerState::TEMP_LOST;
      Check(!run(c, s, 170, 1).output.command.valid, "temporary loss timeout");
      Check(run(c, Sample(180), 180, 2).output.command.valid, "tracking recovery");
    }
    {
      FireControl c(cfg, pcfg);
      auto obs = c.Observe(Sample(0), Feedback(0), At(0));
      for (int i = 0; i < 9; ++i)
        Check(obs.action_mask[i],
              "mask permits all predicted small armor slots without angle heuristics");
      auto s = Sample(0);
      s.prediction.covariance_diagonal[0] = -1;
      obs = c.Observe(s, Feedback(0), At(0));
      Check(std::count(obs.action_mask.begin(), obs.action_mask.end(), true) == 1,
            "invalid estimates only allow wait");
      Check(!run(c, Sample(0), 0, 9).output.command.valid, "invalid action rejected");
    }
    {
      FireControl c(cfg, pcfg);
      FireOnlyPolicyAdapter adapter(cfg);
      auto s = Sample(0);
      auto obs = c.Observe(s, Feedback(0), At(0));
      auto a = adapter.Decide(s, obs, At(0), false);
      Check(a.action == 1, "fire-only initial rule choice");
      s = Sample(10);
      s.prediction.orientation_world = Eigen::Quaterniond::Identity();
      obs = c.Observe(s, Feedback(10), At(10));
      auto b = adapter.Decide(s, obs, At(10), false);
      Check(b.Slot() == 2 && !b.RequestsShot(), "no-shot does not retain old rule slot");
      adapter.RestrictMask(obs, b);
      Check(!obs.action_mask[0] && obs.action_mask[5] && obs.action_mask[6] &&
                std::count(obs.action_mask.begin(), obs.action_mask.end(), true) == 2,
            "fire-only bound mask");
    }
    {
      FireControl a(cfg, pcfg), b(cfg, pcfg), reference(cfg, pcfg);
      for (int ms = 0; ms < 500; ms += 10) {
        auto x = run(a, Sample(ms), ms, ms % 50 == 0 ? 2 : 0).output;
        auto other = Sample(ms);
        other.prediction.center_world.y() = 1;
        static_cast<void>(run(b, other, ms, 4));
        auto y = run(reference, Sample(ms), ms, ms % 50 == 0 ? 2 : 0).output;
        Check(x.command.yaw == y.command.yaw && x.command.pitch == y.command.pitch &&
                  x.command.fire == y.command.fire,
              "interleaved independent instances");
      }
      a.Reset();
      auto x = run(a, Sample(0), 0, 2).output;
      FireControl fresh(cfg, pcfg);
      auto y = run(fresh, Sample(0), 0, 2).output;
      Check(x.command.pitch == y.command.pitch && x.command.fire == y.command.fire,
            "reset replay equals fresh instance");
    }
    {
      ControlSession s(cfg, pcfg);
      auto a = s.Step(Sample(0), {}, true, At(0), 1000000000ULL, PolicyDecision{2});
      Check(a.output.command.valid && a.output.shot_accepted, "session works at zero time");
      s.AcknowledgePublication(a, false, At(0));
      Check(!a.output.published_valid && !a.output.command.fire,
            "failed publication cancels pulse");
      auto b = s.Step(Sample(100), {}, true, At(100), 1100000000ULL, PolicyDecision{0});
      Check(!b.output.command.fire, "failed publication not retried later");
      s.AcknowledgePublication(b, true, At(100));
      bool duplicate = false;
      try {
        s.AcknowledgePublication(b, true, At(100));
      } catch (const std::logic_error&) {
        duplicate = true;
      }
      Check(duplicate, "duplicate acknowledgement rejected");
      auto input = Sample(110);
      input.external_control_enabled = false;
      auto disabled = s.Step(input, {}, true, At(110), 1110000000ULL, PolicyDecision{2});
      Check(!disabled.output.command.valid && !disabled.output.command.fire,
            "control disabled stops");
      s.AcknowledgePublication(disabled, true, At(110));
      auto unhealthy = s.Step(Sample(120), {}, false, At(120), 1120000000ULL, PolicyDecision{2});
      Check(!unhealthy.output.command.valid && !unhealthy.output.command.fire,
            "unhealthy sink stops");
      s.AcknowledgePublication(unhealthy, true, At(120));
    }
    {
      simulation::CombatFrameMeta m;
      m.round_id = 3;
      m.referee_valid = 1;
      m.sim_time_ns = 1000000000;
      m.referee_sample_ns = 950000000;
      m.self_referee.heat_limit = 88;
      m.self_referee.fire_permitted = 1;
      auto o = runtime::AdaptRefereeObservation(&m, 3, At(100), At(120));
      Check(o.valid && std::abs(o.age_at_receive_s - .07) < 1e-12,
            "referee clock domains and processing age");
      m.self_referee.damage_dealt = 999;
      m.self_referee.actual_shots = 888;
      auto v = runtime::AdaptRefereeObservation(&m, 3, At(100), At(120));
      Check(v.heat == o.heat && v.sample_time_ns == o.sample_time_ns,
            "evaluation counters excluded");
      Check(!runtime::AdaptRefereeObservation(&m, 4, At(100), At(120)).valid,
            "referee round mismatch");
      m.referee_sample_ns = m.sim_time_ns + 1;
      Check(!runtime::AdaptRefereeObservation(&m, 3, At(100), At(120)).valid,
            "future referee sample rejected");
    }
    {
      auto p = pcfg;
      p.mpc->max_iterations = 1;
      ControlSession session(cfg, p);
      for (int k = 0; k < 13; ++k) {
        const int MS = 1000 + k * 10;
        auto input = Sample(MS);
        input.prediction.radii_m = {.21, .21};
        input.prediction.armor_tilt_rad = 0;
        input.prediction.center_world = {4., k == 0 ? 0. : 1., -.1127};
        auto result = session.Step(input, {}, true, At(MS),
                                   1000000000ULL + static_cast<std::uint64_t>(k) * 10000000ULL,
                                   PolicyDecision{1});
        if (k == 0)
          Check(result.output.plan.valid, "fallback source solves and publishes");
        else if (k <= 10)
          Check(!result.output.plan.valid && result.output.fallback_active &&
                    result.output.command.valid && !result.output.command.fire,
                "real MPC failure uses finite published trajectory without firing");
        else
          Check(!result.output.command.valid && !result.output.fallback_active,
                "fallback expires after 100ms");
        session.AcknowledgePublication(result, true, At(MS));
      }
    }
    {
      FireControl control(cfg, pcfg);
      auto sample = Sample(0);
      sample.prediction.radii_m[0] = std::numeric_limits<double>::infinity();
      Check(!run(control, sample, 0, 2).output.command.valid,
            "nonfinite geometry rejected before extrapolation");
      auto future = Feedback(1);
      Check(!control.Step(Sample(0), future, At(0), 1, PolicyDecision{2}).output.command.valid,
            "future feedback rejected");
      ControlSession first(cfg, pcfg), second(cfg, pcfg);
      auto a = first.Step(Sample(0), {}, true, At(0), 1, PolicyDecision{1});
      auto b = second.Step(Sample(0), {}, true, At(0), 1, PolicyDecision{1});
      bool foreign = false;
      try {
        first.AcknowledgePublication(b, true, At(0));
      } catch (const std::logic_error&) {
        foreign = true;
      }
      Check(foreign, "foreign session acknowledgement rejected");
      first.AcknowledgePublication(a, true, At(0));
      second.AcknowledgePublication(b, true, At(0));
      bool duplicate_time = false;
      try {
        static_cast<void>(first.Step(Sample(0), {}, true, At(0), 1));
      } catch (const std::invalid_argument&) {
        duplicate_time = true;
      }
      Check(duplicate_time, "nonincreasing session time rejected");
      auto next = Sample(0);
      next.prediction.source_round_id = 2;
      auto reset = first.Step(next, {}, true, At(0), 2, PolicyDecision{2});
      Check(reset.output.shot_accepted, "new round allows zero origin and clears pending history");
      first.AcknowledgePublication(reset, true, At(0));
    }
    {
      // 策略必须在相机位姿投影后运行，且与本周期 MPC 使用同一反馈。
      ControlSession session(cfg, pcfg);
      auto input = Sample(0);
      input.chassis_motion.emplace();
      input.chassis_motion->velocity_body_mps = {1, 0};
      hal::GimbalActuatorTelemetry actuator;
      actuator.valid = true;
      actuator.mode = hal::GimbalActuatorMode::PHYSICAL;
      actuator.state_timestamp_ns = 1020000000;
      actuator.actual_yaw = .1;
      int calls = 0;
      const ControlPolicy POLICY = [&](const ControlInputSnapshot& projected,
                                       const PolicyObservation& obs) {
        ++calls;
        Check(std::abs(projected.world_t_gimbal.translation.x() - .02) < 1e-12,
              "policy receives projected muzzle origin");
        Check(
            std::abs(obs.feedback.yaw - .1) < 1e-12 && std::abs(obs.prediction_age_s - .02) < 1e-12,
            "policy receives current fused feedback and source age");
        Check(obs.previous_slot == -1 && obs.action_mask[2], "initial callback mask");
        return PolicyDecision{2};
      };
      auto result = session.StepWithPolicy(input, actuator, true, At(20), 1020000000, POLICY);
      Check(calls == 1 && result.decision && result.decision->action == 2 &&
                result.output.shot_accepted,
            "same-cycle callback decision executed once");
      bool early_ack = false;
      try {
        session.AcknowledgePublication(result, true, At(19));
      } catch (const std::invalid_argument&) {
        early_ack = true;
      }
      Check(early_ack, "acknowledgement cannot predate step");
      session.AcknowledgePublication(result, true, At(20));
      input = Sample(30);
      input.prediction.track_generation = 2;
      result = session.StepWithPolicy(
          input, {}, true, At(30), 1030000000,
          [&](const ControlInputSnapshot&, const PolicyObservation& obs) {
            Check(obs.previous_slot == -1 && !obs.action_mask[2],
                  "new target clears slot before observation but preserves shot interval");
            return PolicyDecision{1};
          });
      session.AcknowledgePublication(result, true, At(30));
      input = Sample(0);
      input.prediction.source_round_id = 2;
      result = session.StepWithPolicy(
          input, {}, true, At(0), 2000000000,
          [&](const ControlInputSnapshot&, const PolicyObservation& obs) {
            Check(obs.previous_slot == -1 && !obs.since_request_s && obs.action_mask[2],
                  "round reset precedes callback observation");
            return PolicyDecision{2};
          });
      session.AcknowledgePublication(result, true, At(0));
      input = Sample(10);
      input.prediction.source_round_id = 2;
      result = session.StepWithPolicy(
          input, {}, false, At(10), 2010000000,
          [&](const ControlInputSnapshot&, const PolicyObservation& obs) {
            for (int i = 2; i <= 8; i += 2)
              Check(!obs.action_mask[i], "unhealthy transport masks shooting");
            return PolicyDecision{2};
          });
      Check(!result.output.command.fire && !result.output.shot_accepted,
            "unhealthy transport never admits a pulse");
      session.AcknowledgePublication(result, false, At(10));
    }
    {
      FireControl control(cfg, pcfg);
      static_cast<void>(run(control, Sample(0), 0, 2));
      auto input = Sample(10);
      input.referee.valid = false;
      Check(!run(control, input, 10, 0).output.command.fire, "referee loss cancels active pulse");
      Check(!run(control, Sample(20), 20, 0).output.command.fire,
            "referee recovery does not replay canceled pulse");
      simulation::CombatFrameMeta meta;
      meta.round_id = 1;
      meta.referee_valid = 1;
      meta.self_referee.heat_limit = 88;
      meta.self_referee.heat = std::numeric_limits<float>::quiet_NaN();
      Check(!runtime::AdaptRefereeObservation(&meta, 1, At(0), At(0)).valid,
            "malformed referee values invalidated at whitelist boundary");
    }
    std::cout << "PASS " << checks << " checks\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL after " << checks << ": " << e.what() << '\n';
    return 1;
  }
}

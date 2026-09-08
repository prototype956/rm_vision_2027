#include "modules/fire_control/control_session.hpp"
#include "modules/fire_control/fire_only_policy_adapter.hpp"
#include "runtime/referee_observation_adapter.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

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
int main(int argc, char** argv) {
  try {
    if (argc != 2)
      return 2;
    auto cfg = ParseFireControlConfig(YAML::LoadFile(std::string(argv[1]) + "/fire_control.yaml"));
    auto pcfg = ParseGimbalTrajectoryPlannerConfig(
        YAML::LoadFile(std::string(argv[1]) + "/gimbal_trajectory_planner.yaml"));
    cfg.auto_fire = true;
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
      Check(run(c, Sample(410), 410, 2).output.shot_accepted, "referee recovery");
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
      p.max_iterations = 1;
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

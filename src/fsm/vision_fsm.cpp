/**
 * @file vision_fsm.cpp
 * @brief 视觉系统状态机实现
 *
 * 每个状态的 on_enter / on_exit / on_update 集中在 handlers 命名空间，
 * 结构与 vision_fsm.hpp 中的前向声明一一对应。
 *
 * 【状态转换一览】
 *
 *  IDLE      → INIT          由 VisionFSM::Start() 触发
 *  INIT      → AUTO_AIM      Pipeline.IsRunning() 且稳定时间到
 *  AUTO_AIM  → ERROR         pipeline->CheckErrors() 为真
 *  AUTO_AIM  → ENERGY_BUFF   上行 mode 字段 = 1 (预留，当前未实现)
 *  ENERGY_BUFF → AUTO_AIM    上行 mode 字段 = 0 (预留)
 *  ENERGY_BUFF → ERROR       pipeline->CheckErrors() 为真
 *  ERROR     → RECOVERY      等待 ERROR_WAIT_MS 后自动触发（次数限制内）
 *  RECOVERY  → INIT          Reset() 完成后
 *  任意       → IDLE          VisionFSM::Stop() 触发
 */

#include "vision_fsm.hpp"

#include "core/logger.hpp"

#include <chrono>
#include <stdexcept>

namespace mv::fsm {

// ============================================================================
// 工具宏（仅此文件内部）
// ============================================================================

// ctx.has_pending_transition = true; ctx.requested_state = state;
// 封装为 inline 避免宏副作用
static inline void RequestState(SystemContext& ctx, SystemState state) {
  ctx.requested_state = state;
  ctx.has_pending_transition = true;
}

// ============================================================================
// 状态处理器实现
// ============================================================================

namespace handlers {

// ── IDLE ─────────────────────────────────────────────────────────────────────

void IdleOnEnter(SystemContext& ctx) {
  // 确保 pipeline 停止（Stop() 幂等）
  if (ctx.pipeline && ctx.pipeline->IsRunning()) {
    ctx.pipeline->Stop();
  }
  MV_LOG_INFO("FSM", "进入 IDLE 状态，Pipeline 已停止");
}

void IdleOnUpdate(SystemContext& ctx) {
  // IDLE 是被动状态，等待外部 Start() 触发 Transition(INIT)
  (void)ctx;
}

// ── INIT ─────────────────────────────────────────────────────────────────────

void InitOnEnter(SystemContext& ctx) {
  if (ctx.pipeline == nullptr) {
    throw std::logic_error("InitOnEnter: pipeline 为空");
  }
  MV_LOG_INFO("FSM", "进入 INIT 状态，正在启动 Pipeline...");
  ctx.pipeline->Start();
  ctx.init_enter_time = std::chrono::steady_clock::now();
}

void InitOnUpdate(SystemContext& ctx) {
  // 等待稳定时间，同时检查 pipeline 是否成功启动
  auto elapsed = std::chrono::steady_clock::now() - ctx.init_enter_time;
  if (elapsed < SystemContext::INIT_STABILIZE_MS) {
    return;  // 尚在稳定等待期
  }

  if (ctx.pipeline->CheckErrors()) {
    MV_LOG_ERROR("FSM", "INIT 阶段 Pipeline 出错，转换到 ERROR");
    RequestState(ctx, SystemState::ERROR);
    return;
  }

  if (ctx.pipeline->IsRunning()) {
    MV_LOG_INFO("FSM", "Pipeline 启动成功，转换到 AUTO_AIM");
    RequestState(ctx, SystemState::AUTO_AIM);
  } else {
    // 已超过稳定期但尚未全部运行，给多一次机会
    MV_LOG_WARN("FSM", "Pipeline 尚未全部运行（frame_ch={}, detect_ch={}, control_ch={}）",
                ctx.pipeline->FrameChannelSize(), ctx.pipeline->DetectChannelSize(),
                ctx.pipeline->ControlChannelSize());
  }
}

// ── AUTO_AIM ─────────────────────────────────────────────────────────────────

void AutoAimOnEnter(SystemContext& ctx) {
  MV_LOG_INFO("FSM", "进入 AUTO_AIM 模式，enemy_color={}",
              ctx.pipeline->State().enemy_color.load() == ArmorColor::RED ? "RED" : "BLUE");
}

void AutoAimOnUpdate(SystemContext& ctx) {
  if (ctx.pipeline->CheckErrors()) {
    MV_LOG_ERROR("FSM", "AUTO_AIM 检测到 Pipeline 错误，转换到 ERROR");
    RequestState(ctx, SystemState::ERROR);
    return;
  }

  // 预留：读取上行 mode，决定是否切换到打符
  // auto mode = ctx.pipeline->State().mode.load();
  // if (mode == 1) { RequestState(ctx, SystemState::ENERGY_BUFF); }
}

void AutoAimOnExit(SystemContext& ctx) {
  MV_LOG_INFO("FSM", "离开 AUTO_AIM 模式");
  (void)ctx;
}

// ── ENERGY_BUFF ───────────────────────────────────────────────────────────────

void EnergyBuffOnEnter(SystemContext& ctx) {
  MV_LOG_INFO("FSM", "进入 ENERGY_BUFF 模式（预留，当前行为与 AUTO_AIM 相同）");
  (void)ctx;
}

void EnergyBuffOnUpdate(SystemContext& ctx) {
  if (ctx.pipeline->CheckErrors()) {
    MV_LOG_ERROR("FSM", "ENERGY_BUFF 检测到 Pipeline 错误，转换到 ERROR");
    RequestState(ctx, SystemState::ERROR);
    return;
  }

  // 预留：读取上行 mode，决定是否切换回自瞄
  // if (mode == 0) { RequestState(ctx, SystemState::AUTO_AIM); }
}

void EnergyBuffOnExit(SystemContext& ctx) {
  MV_LOG_INFO("FSM", "离开 ENERGY_BUFF 模式");
  (void)ctx;
}

// ── ERROR ─────────────────────────────────────────────────────────────────────

void ErrorOnEnter(SystemContext& ctx) {
  // 停止 pipeline，记录时间和错误信息
  if (ctx.pipeline->IsRunning()) {
    ctx.pipeline->Stop();
  }
  ctx.error_enter_time = std::chrono::steady_clock::now();

  // 收集各节点错误码（当前简单打印，Stage 6 可精细化）
  MV_LOG_ERROR("FSM", "进入 ERROR 状态，Pipeline 已停止。恢复次数={}/{}", ctx.recovery_attempts,
               SystemContext::MAX_RECOVERY);
}

void ErrorOnUpdate(SystemContext& ctx) {
  auto elapsed = std::chrono::steady_clock::now() - ctx.error_enter_time;
  if (elapsed < SystemContext::ERROR_WAIT_MS) {
    return;  // 冷却等待期
  }

  if (ctx.recovery_attempts >= SystemContext::MAX_RECOVERY) {
    MV_LOG_CRITICAL("FSM", "已达最大恢复次数 {}，系统停留在 ERROR 状态，请人工干预",
                    SystemContext::MAX_RECOVERY);
    return;
  }

  MV_LOG_WARN("FSM", "尝试自动恢复（第 {} 次）", ctx.recovery_attempts + 1);
  RequestState(ctx, SystemState::RECOVERY);
}

// ── RECOVERY ─────────────────────────────────────────────────────────────────

void RecoveryOnEnter(SystemContext& ctx) {
  ctx.recovery_attempts++;
  MV_LOG_INFO("FSM", "进入 RECOVERY，执行 Pipeline::Reset()（第 {} 次恢复）",
              ctx.recovery_attempts);
  ctx.pipeline->Reset();
}

void RecoveryOnUpdate(SystemContext& ctx) {
  // Reset() 是同步的，完成后立即重新进入 INIT
  MV_LOG_INFO("FSM", "Reset 完成，重新 INIT");
  RequestState(ctx, SystemState::INIT);
}

}  // namespace handlers

// ============================================================================
// VisionFSM 实现
// ============================================================================

VisionFSM::VisionFSM(std::unique_ptr<pipeline::VisionPipeline> pipeline) {
  if (pipeline == nullptr) {
    throw std::invalid_argument("VisionFSM: pipeline 不可为空");
  }
  ctx_.pipeline = std::move(pipeline);
  RegisterHandlers();
  sm_.Init(SystemState::IDLE, ctx_);
}

VisionFSM::~VisionFSM() {
  // 确保析构时 pipeline 干净停止
  if (ctx_.pipeline && ctx_.pipeline->IsRunning()) {
    ctx_.pipeline->Stop();
  }
}

void VisionFSM::RegisterHandlers() {
  using S = SystemState;
  using H = StateMachine<S, SystemContext>::StateHandler;

  sm_.Register(S::IDLE, H{
                            .on_enter = handlers::IdleOnEnter,
                            .on_exit = nullptr,
                            .on_update = handlers::IdleOnUpdate,
                        });

  sm_.Register(S::INIT, H{
                            .on_enter = handlers::InitOnEnter,
                            .on_exit = nullptr,
                            .on_update = handlers::InitOnUpdate,
                        });

  sm_.Register(S::AUTO_AIM, H{
                                .on_enter = handlers::AutoAimOnEnter,
                                .on_exit = handlers::AutoAimOnExit,
                                .on_update = handlers::AutoAimOnUpdate,
                            });

  sm_.Register(S::ENERGY_BUFF, H{
                                   .on_enter = handlers::EnergyBuffOnEnter,
                                   .on_exit = handlers::EnergyBuffOnExit,
                                   .on_update = handlers::EnergyBuffOnUpdate,
                               });

  sm_.Register(S::ERROR, H{
                             .on_enter = handlers::ErrorOnEnter,
                             .on_exit = nullptr,
                             .on_update = handlers::ErrorOnUpdate,
                         });

  sm_.Register(S::RECOVERY, H{
                                .on_enter = handlers::RecoveryOnEnter,
                                .on_exit = nullptr,
                                .on_update = handlers::RecoveryOnUpdate,
                            });
}

void VisionFSM::Start() {
  if (sm_.Current() != SystemState::IDLE) {
    MV_LOG_WARN("FSM", "Start() 被调用时状态为 {}，忽略", SystemStateName(sm_.Current()));
    return;
  }
  ctx_.recovery_attempts = 0;
  sm_.Transition(SystemState::INIT, ctx_);
}

void VisionFSM::Stop() {
  if (sm_.Current() == SystemState::IDLE) {
    return;
  }
  MV_LOG_INFO("FSM", "Stop() 调用，强制转换到 IDLE（当前={}）", SystemStateName(sm_.Current()));
  sm_.Transition(SystemState::IDLE, ctx_);
}

void VisionFSM::Update() {
  // 1. 驱动当前状态的 on_update（内部可能写 has_pending_transition）
  sm_.Update(ctx_);

  // 2. 处理 pending transition（由 on_update 内部请求）
  if (ctx_.has_pending_transition) {
    ctx_.has_pending_transition = false;
    SystemState next = ctx_.requested_state;
    MV_LOG_DEBUG("FSM", "状态转换: {} → {}", SystemStateName(sm_.Current()), SystemStateName(next));
    sm_.Transition(next, ctx_);
  }
}

void VisionFSM::RequestTransition(SystemState next) {
  ctx_.requested_state = next;
  ctx_.has_pending_transition = true;
}

void VisionFSM::Recover() {
  RequestTransition(SystemState::RECOVERY);
}

SystemState VisionFSM::CurrentState() const noexcept {
  return sm_.Current();
}

bool VisionFSM::IsRunning() const noexcept {
  auto cur = sm_.Current();
  return cur == SystemState::AUTO_AIM || cur == SystemState::ENERGY_BUFF;
}

bool VisionFSM::HasError() const noexcept {
  return sm_.Current() == SystemState::ERROR;
}

int VisionFSM::LastErrorCode() const noexcept {
  return ctx_.last_error_code;
}

const pipeline::VisionPipeline* VisionFSM::Pipeline() const noexcept {
  return ctx_.pipeline.get();
}

}  // namespace mv::fsm

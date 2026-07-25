/**
 * @file pipeline.hpp
 * @brief 视觉流水线编排器 (VisionPipeline)
 *
 * 【职责】
 *   VisionPipeline 是 Stage 4 的顶层类，负责：
 *   1. 持有所有 Channel（决定缓冲队列的生命周期）；
 *   2. 持有所有 Node（保证析构顺序安全）；
 *   3. 提供统一的 Start() / Stop() / Reset() 接口；
 *   4. 暴露 SharedState（ArmorColor 等原子状态）供外部读写；
 *   5. 提供 CheckErrors() 给状态机定期轮询（Stage 5）。
 *
 * 【构建方式（Builder 模式）】
 *   VisionPipeline 通过 Builder 构造，隔离"依赖注入"逻辑：
 * @code
 *   auto pipeline = VisionPipeline::Builder{}
 *     .Camera(Factory<ICamera>::Create("mindvision"))
 *     .Detector(Factory<IDetector>::Create("basic"))
 *     .Solver(Factory<ISolver>::Create("pnp"))
 *     .Predictor(Factory<IPredictor>::Create("ekf"))
 *     .Voter(Factory<IVoter>::Create("cooldown"))
 *     .Serial(Factory<ISerial>::Create("uart"))
 *     .Shooter(Factory<IShooter>::Create("rm"))
 *     .Build();
 *
 *   pipeline.Start();
 *   // ...
 *   pipeline.Stop();
 * @endcode
 *
 * 【Channel 容量配置（影响实时性）】
 *   - frame_ch_:   容量 2 — Capture 快，Detect 慢时丢帧（实时优先）
 *   - detect_ch_:  容量 2 — Detect 快，Predict 慢时丢帧
 *   - control_ch_: 容量 1 — 只保留最新一帧控制量（串口发送慢时）
 *
 * 【停止顺序（防止死锁）】
 *   Stop() 必须按如下顺序执行，否则后级 Pop 阻塞后级线程无法退出：
 *   1. CaptureNode::Stop()  → 关闭 frame_ch_
 *   2. DetectNode::Stop()   → 关闭 detect_ch_
 *   3. PredictNode::Stop()  → 关闭 control_ch_
 *   4. SerialNode::Stop()
 *
 * 【错误处理接口】
 *   Stage 5 状态机代码：
 * @code
 *   if (pipeline_.CheckErrors()) {
 *     state_machine_.Transition(SystemState::ERROR);
 *   }
 * @endcode
 */
#pragma once

#include "capture_node.hpp"
#include "channel.hpp"
#include "detect_node.hpp"
#include "hal/camera/i_camera.hpp"
#include "hal/serial/i_serial.hpp"
#include "interfaces/i_detector.hpp"
#include "interfaces/i_predictor.hpp"
#include "interfaces/i_shooter.hpp"
#include "interfaces/i_solver.hpp"
#include "interfaces/i_voter.hpp"
#include "interfaces/types.hpp"
#include "packet.hpp"
#include "predict_node.hpp"
#include "serial_node.hpp"
#include "shared_state.hpp"

#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>

namespace mv::pipeline {

/**
 * @brief 视觉流水线主类
 *
 * 拥有所有节点和通道的所有权。
 * 不可拷贝/移动（持有互斥量和线程）。
 */
class VisionPipeline {
 public:
  // ── 共享状态（节点间通过原子变量交换的运行时状态）───────────────────────

  /**
   * @brief 节点间共享的原子状态
   *
   * 由 SerialNode 写入，由 DetectNode / PredictNode 读取。
   * 使用 std::atomic 保证无锁读写，不需要 mutex。
   */
  // SharedState 定义已提取到 shared_state.hpp，此处使用类型别名便于代码可读性
  using SharedState = mv::pipeline::SharedState;

  // ── 构造 / 析构 ───────────────────────────────────────────────────────────

  VisionPipeline(const VisionPipeline&) = delete;
  VisionPipeline& operator=(const VisionPipeline&) = delete;
  VisionPipeline(VisionPipeline&&) = delete;
  VisionPipeline& operator=(VisionPipeline&&) = delete;

  ~VisionPipeline() { Stop(); }

  // ── 生命周期 ──────────────────────────────────────────────────────────────

  /**
   * @brief 按正确顺序启动所有节点线程
   *
   * 启动顺序：SerialNode → PredictNode → DetectNode → CaptureNode（末级先启动）
   * 这样保证下游线程就绪后上游才开始产生数据，避免初始帧被丢弃。
   */
  void Start() {
    serial_node_->Start();
    predict_node_->Start();
    detect_node_->Start();
    capture_node_->Start();
  }

  /**
   * @brief 按正确顺序停止所有节点线程（阻塞直到全部退出）
   *
   * 停止顺序：CaptureNode → DetectNode → PredictNode → SerialNode（前级先停）
   * 每个节点的 Stop() 会 Shutdown 其输出通道，触发下一级退出阻塞。
   */
  void Stop() {
    if (capture_node_) {
      capture_node_->Stop();
    }
    if (detect_node_) {
      detect_node_->Stop();
    }
    if (predict_node_) {
      predict_node_->Stop();
    }
    if (serial_node_) {
      serial_node_->Stop();
    }
  }

  /**
   * @brief 重置流水线（清空通道 + 重新可以 Start）
   *
   * 必须在 Stop() 之后调用。
   */
  void Reset() {
    frame_ch_->Reset();
    detect_ch_->Reset();
    control_ch_->Reset();
  }

  // ── 诊断 ─────────────────────────────────────────────────────────────────

  /**
   * @brief 检查所有节点是否发生错误
   * @return true = 至少有一个节点发生不可恢复的错误
   *
   * Stage 5 状态机调用此方法触发 ERROR 状态转换。
   */
  [[nodiscard]] bool CheckErrors() const noexcept {
    return capture_node_->HasError() || detect_node_->HasError() || predict_node_->HasError() ||
           serial_node_->HasError();
  }

  /** @return 各节点是否全部在运行 */
  [[nodiscard]] bool IsRunning() const noexcept {
    return capture_node_->IsRunning() && detect_node_->IsRunning() && predict_node_->IsRunning() &&
           serial_node_->IsRunning();
  }

  /** @return 共享状态引用（用于外部读写  enemy_color 等）*/
  [[nodiscard]] SharedState& State() noexcept { return state_; }
  [[nodiscard]] const SharedState& State() const noexcept { return state_; }

  // ── 通道诊断 ─────────────────────────────────────────────────────────────

  /** @return 各通道当前积压帧数（用于延迟诊断）*/
  [[nodiscard]] size_t FrameChannelSize() const { return frame_ch_->Size(); }
  [[nodiscard]] size_t DetectChannelSize() const { return detect_ch_->Size(); }
  [[nodiscard]] size_t ControlChannelSize() const { return control_ch_->Size(); }

  // ── Builder ──────────────────────────────────────────────────────────────

  /**
   * @brief 流水线构建器
   *
   * 强制调用方显式提供所有依赖，防止漏注入导致空指针崩溃。
   * Build() 在满足所有必要依赖后才构造 VisionPipeline。
   */
  class Builder {
   public:
    Builder& Camera(std::unique_ptr<hal::ICamera> cam) {
      camera_ = std::move(cam);
      return *this;
    }
    Builder& Detector(std::unique_ptr<IDetector> det) {
      detector_ = std::move(det);
      return *this;
    }
    Builder& Solver(std::unique_ptr<ISolver> sol) {
      solver_ = std::move(sol);
      return *this;
    }
    Builder& Predictor(std::unique_ptr<IPredictor> pred) {
      predictor_ = std::move(pred);
      return *this;
    }
    Builder& Voter(std::unique_ptr<IVoter> voter) {
      voter_ = std::move(voter);
      return *this;
    }
    Builder& Serial(std::unique_ptr<hal::ISerial> serial) {
      serial_ = std::move(serial);
      return *this;
    }
    Builder& Shooter(std::unique_ptr<IShooter> shooter) {
      shooter_ = std::move(shooter);
      return *this;
    }

    /** 通道容量配置（可选，使用默认值）*/
    Builder& FrameChannelCapacity(size_t cap) {
      frame_cap_ = cap;
      return *this;
    }
    Builder& DetectChannelCapacity(size_t cap) {
      detect_cap_ = cap;
      return *this;
    }
    Builder& ControlChannelCapacity(size_t cap) {
      control_cap_ = cap;
      return *this;
    }

    /**
     * @brief 构造 VisionPipeline
     * @throws std::invalid_argument  任何必要依赖为 nullptr 时抛出
     */
    [[nodiscard]] std::unique_ptr<VisionPipeline> Build();

   private:
    std::unique_ptr<hal::ICamera> camera_;
    std::unique_ptr<IDetector> detector_;
    std::unique_ptr<ISolver> solver_;
    std::unique_ptr<IPredictor> predictor_;
    std::unique_ptr<IVoter> voter_;
    std::unique_ptr<hal::ISerial> serial_;
    std::unique_ptr<IShooter> shooter_;

    size_t frame_cap_{2};
    size_t detect_cap_{2};
    size_t control_cap_{1};
  };

 private:
  // 私有构造（只能通过 Builder 创建）
  VisionPipeline() = default;

  SharedState state_;

  // 通道（生命周期由 Pipeline 管理）
  std::shared_ptr<Channel<FramePacket>> frame_ch_;
  std::shared_ptr<Channel<DetectPacket>> detect_ch_;
  std::shared_ptr<Channel<ControlPacket>> control_ch_;

  // 节点（声明顺序 = 析构顺序的逆序：逆向清理）
  std::unique_ptr<SerialNode> serial_node_;
  std::unique_ptr<PredictNode> predict_node_;
  std::unique_ptr<DetectNode> detect_node_;
  std::unique_ptr<CaptureNode> capture_node_;

  friend class Builder;
};

// ── Builder::Build() 实现 ────────────────────────────────────────────────────

inline std::unique_ptr<VisionPipeline> VisionPipeline::Builder::Build() {
  // 检查必要依赖
  const auto CHECK = [](const auto& ptr, const char* name) {
    if (!ptr) {
      throw std::invalid_argument(std::string("VisionPipeline::Builder: missing ") + name);
    }
  };
  CHECK(camera_, "Camera");
  CHECK(detector_, "Detector");
  CHECK(solver_, "Solver");
  CHECK(predictor_, "Predictor");
  CHECK(voter_, "Voter");
  CHECK(serial_, "Serial");
  CHECK(shooter_, "Shooter");

  // 使用 new + private constructor（unique_ptr 无法直接访问私有构造）
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
  auto pipeline = std::unique_ptr<VisionPipeline>(new VisionPipeline{});

  // 创建通道
  pipeline->frame_ch_ = std::make_shared<Channel<FramePacket>>(frame_cap_);
  pipeline->detect_ch_ = std::make_shared<Channel<DetectPacket>>(detect_cap_);
  pipeline->control_ch_ = std::make_shared<Channel<ControlPacket>>(control_cap_);

  // 创建节点（注入依赖）
  pipeline->capture_node_ = std::make_unique<CaptureNode>(std::move(camera_), pipeline->frame_ch_);

  pipeline->detect_node_ =
      std::make_unique<DetectNode>(std::move(detector_), std::move(solver_), pipeline->frame_ch_,
                     pipeline->detect_ch_, pipeline->state_);

  pipeline->predict_node_ =
      std::make_unique<PredictNode>(std::move(predictor_), std::move(voter_), pipeline->detect_ch_,
                                    pipeline->control_ch_, pipeline->state_);

  pipeline->serial_node_ = std::make_unique<SerialNode>(std::move(serial_), std::move(shooter_),
                                                        pipeline->control_ch_, pipeline->state_);

  return pipeline;
}

}  // namespace mv::pipeline

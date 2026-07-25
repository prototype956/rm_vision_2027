/**
 * @file factory.hpp
 * @brief 模板工厂注册表 (Factory<Base>)
 *
 * 【设计目标】
 *   消除"只要修改算法实现就必须修改 main.cpp"的问题。
 *   旧代码 main.cpp：
 *     if (mode == "basic") detector = std::make_unique<BasicArmorDetector>(...);
 *     else if (mode == "dnn") detector = std::make_unique<DnnArmorDetector>(...);
 *   每次新增实现都要修改 main，违反开闭原则。
 *
 *   新方式（注册 + 按名创建）：
 *     // 在各自的 .cpp 注册（一次性）
 *     MV_REGISTER_DETECTOR("basic", BasicArmorDetector);
 *     MV_REGISTER_DETECTOR("dnn",   DnnArmorDetector);
 *
 *     // main.cpp 只需
 *     auto detector = mv::Factory<mv::IDetector>::Create(cfg.Get<std::string>("detector.type"));
 *     新增第三种检测器时，main.cpp 零修改。
 *
 * 【线程安全性】
 *   Register() 通常在 main() 开始前通过全局静态初始化调用，
 *   此时只有一个线程运行，无需加锁。
 *   Create()/Keys() 只读访问 map，并发安全（map 写入已结束）。
 *   若需要运行时动态注册，调用方自行加锁。
 *
 * 【使用方式】
 * @code
 *   // ── 注册（推荐放在实现 .cpp 的文件作用域）────────────────────────────
 *   // 使用便捷宏（需要先 #define 对应的宏，见下方）
 *   MV_REGISTER_DETECTOR("basic", BasicArmorDetector);
 *
 *   // 或手动注册
 *   MV_FACTORY_REGISTER(mv::IDetector, "basic",
 *       []{ return std::make_unique<BasicArmorDetector>(); });
 *
 *   // ── 创建（在 main / Pipeline 中）────────────────────────────────────
 *   auto detector = mv::Factory<mv::IDetector>::Create("basic");
 *   if (!detector) { // "basic" 未注册 }
 *
 *   // ── 查询已注册的键 ────────────────────────────────────────────────────
 *   for (auto& key : mv::Factory<mv::IDetector>::Keys()) { ... }
 * @endcode
 */
#pragma once

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace mv {

/**
 * @brief 泛型工厂注册表
 *
 * @tparam Base  接口基类（IDetector、ISolver、IPredictor、ICamera、ISerial...）
 */
template <typename Base>
class Factory {
 public:
  /** 构造函数类型：无参，返回 unique_ptr<Base> */
  using Creator = std::function<std::unique_ptr<Base>()>;

  // ── 注册 ─────────────────────────────────────────────────────────────────

  /**
   * @brief 注册一个具体实现
   * @param key      字符串键（来自 YAML 配置的 type 字段，如 "basic"、"dnn"）
   * @param creator  工厂函数（通常是 lambda: []{ return make_unique<ConcreteType>(); }）
   *
   * 重复注册同一 key 会覆盖原有注册（方便测试时替换实现）。
   */
  static void Register(const std::string& key, Creator creator) {
    Registry()[key] = std::move(creator);
  }

  // ── 创建 ─────────────────────────────────────────────────────────────────

  /**
   * @brief 按 key 创建实例
   * @param key  注册时使用的字符串键
   * @return     unique_ptr<Base>，如果 key 未注册则返回 nullptr
   *
   * 返回 nullptr 而不抛异常，调用方可以打印友好错误信息后退出，
   * 比 catch 异常更直观。
   */
  [[nodiscard]] static std::unique_ptr<Base> Create(const std::string& key) {
    auto& reg = Registry();
    const auto FOUND = reg.find(key);
    if (FOUND == reg.end()) {
      return nullptr;
    }
    return FOUND->second();
  }

  // ── 查询 ─────────────────────────────────────────────────────────────────

  /**
   * @brief 返回所有已注册的键（用于启动时打印可用实现列表）
   */
  [[nodiscard]] static std::vector<std::string> Keys() {
    std::vector<std::string> keys;
    keys.reserve(Registry().size());
    for (const auto& [key, _] : Registry()) {
      keys.push_back(key);
    }
    return keys;
  }

  /** @return key 是否已注册 */
  [[nodiscard]] static bool Has(const std::string& key) { return Registry().count(key) > 0; }

 private:
  // Meyers Singleton：第一次调用时构造，保证静态初始化顺序安全
  static auto& Registry() {
    static std::unordered_map<std::string, Creator> registry;
    return registry;
  }
};

// ============================================================================
// 便捷注册宏（利用全局静态变量在 main() 之前完成注册）
// ============================================================================

/**
 * @brief 通用注册宏（用于任意接口类型）
 *
 * @param Base      接口类型，如 mv::IDetector
 * @param key_str   字符串键，如 "basic"
 * @param ConcreteT 具体实现类型，如 BasicArmorDetector
 *
 * 使用匿名命名空间内的静态变量触发注册，避免符号冲突。
 */
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MV_FACTORY_REGISTER(Base, key_str, ConcreteT)                                     \
  namespace {                                                                             \
  const bool MV_FACTORY_REGISTERED_##ConcreteT = [] {                                     \
    ::mv::Factory<Base>::Register(key_str, [] { return std::make_unique<ConcreteT>(); }); \
    return true;                                                                          \
  }();                                                                                    \
  }  // namespace

}  // namespace mv

// ============================================================================
// 各接口专属注册宏（语义更清晰，推荐使用这些）
// ============================================================================

// 使用前需 #include 对应的接口头文件

/** 注册 IDetector 实现：MV_REGISTER_DETECTOR("basic", BasicArmorDetector) */
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MV_REGISTER_DETECTOR(key_str, ConcreteT) \
  MV_FACTORY_REGISTER(::mv::IDetector, key_str, ConcreteT)

/** 注册 ISolver 实现：MV_REGISTER_SOLVER("pnp", BasicPnpSolver) */
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MV_REGISTER_SOLVER(key_str, ConcreteT) \
  MV_FACTORY_REGISTER(::mv::ISolver, key_str, ConcreteT)

/** 注册 IPredictor 实现：MV_REGISTER_PREDICTOR("ekf", EkfPredictor) */
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MV_REGISTER_PREDICTOR(key_str, ConcreteT) \
  MV_FACTORY_REGISTER(::mv::IPredictor, key_str, ConcreteT)

/** 注册 IVoter 实现：MV_REGISTER_VOTER("cooldown", CooldownVoter) */
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MV_REGISTER_VOTER(key_str, ConcreteT) \
  MV_FACTORY_REGISTER(::mv::IVoter, key_str, ConcreteT)

/** 注册 IShooter 实现：MV_REGISTER_SHOOTER("rm", RmShooter) */
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MV_REGISTER_SHOOTER(key_str, ConcreteT) \
  MV_FACTORY_REGISTER(::mv::IShooter, key_str, ConcreteT)

/** 注册 ICamera 实现：MV_REGISTER_CAMERA("mindvision", MindVisionCamera) */
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MV_REGISTER_CAMERA(key_str, ConcreteT) \
  MV_FACTORY_REGISTER(::mv::hal::ICamera, key_str, ConcreteT)

/** 注册 ISerial 实现：MV_REGISTER_SERIAL("uart", UartSerial) */
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MV_REGISTER_SERIAL(key_str, ConcreteT) \
  MV_FACTORY_REGISTER(::mv::hal::ISerial, key_str, ConcreteT)

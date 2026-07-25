/**
 * @file config.hpp
 * @brief 配置管理器
 *
 *   使用单例比全局变量多一层懒初始化保证（第一次 Instance() 时构造），
 *
 *   使用 YAML 支持注释，方便调参时在文件里写备注；
 *
 *   配置在运行时几乎只读（写只发生在启动和热重载两个时机），
 *   shared_mutex 允许多线程并发读，只有 Load/Reload 时独占写，
 *
 * 【快速上手】
 * @code
 *   auto& cfg = mv::ConfigManager::Instance();
 *
 *   // 启动时加载（通常在 main 里做一次）
 *   cfg.Load("configs/vision.yaml");                     // 根命名空间
 *   cfg.Load("configs/armor/basic_armor.yaml", "armor"); // armor 命名空间
 *
 *   // 读取：找不到时返回默认值，绝不抛异常
 *   auto color  = cfg.Get<std::string>("enemy_color", "red");
 *   auto thresh = cfg.Get<int>("armor.light_thresh", 100);
 *
 *   // 读取：必须存在，不存在抛 std::out_of_range
 *   auto fps = cfg.GetRequired<int>("camera.fps");
 *
 *   // 检查 key 是否存在
 *   if (cfg.Has("armor.debug_mode")) { ... }
 *
 *   // 热重载（如接收到 SIGUSR1 时调用）
 *   cfg.Reload();
 * @endcode
 */
#pragma once

#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace mv {

class ConfigManager {
 public:
  // ── 单例访问 ─────────────────────────────────────────────────────────────
  static ConfigManager& Instance() {
    static ConfigManager inst;
    return inst;
  }

  // 禁止拷贝 / 移动
  ConfigManager(const ConfigManager&) = delete;
  ConfigManager& operator=(const ConfigManager&) = delete;
  ConfigManager(ConfigManager&&) = delete;
  ConfigManager& operator=(ConfigManager&&) = delete;
  ~ConfigManager() = default;

  // ── 加载接口 ──────────────────────────────────────────────────────────────

  /**
   * @brief 加载一个 YAML 文件，合并到指定命名空间
   *
   * @param file_path    YAML 文件路径（相对路径以进程工作目录为基准）
   * @param name_space   命名空间前缀。留空则合并到根节点，
   *                     "armor" 则文件中的 "foo" 通过 "armor.foo" 访问。
   * @throw YAML::BadFile         文件不存在或无权限读取
   * @throw YAML::ParserException YAML 语法错误
   */
  void Load(const std::string& file_path, const std::string& name_space = "") {
    std::unique_lock<std::shared_mutex> lock(rw_mutex_);
    YAML::Node incoming = YAML::LoadFile(file_path);
    auto& target = namespaces_[name_space];
    FlatMerge(target, incoming);
    loaded_files_.emplace_back(file_path, name_space);
  }

  /**
   * @brief 热重载：重新读取所有已加载的文件，刷新内存中的配置
   *
   * @note 重载期间持有独占写锁，不要在图像处理主循环里调用，
   *       建议在独立的管理线程中接收信号后调用（如 SIGUSR1）。
   * @throw YAML::Exception 任意文件解析失败时抛出，原配置保持不变
   */
  void Reload() {
    std::unique_lock<std::shared_mutex> lock(rw_mutex_);
    // 先在临时副本上重建，成功后才替换，避免部分失败导致数据丢失
    std::unordered_map<std::string, YAML::Node> new_namespaces;
    for (const auto& [path, name_space] : loaded_files_) {
      YAML::Node incoming = YAML::LoadFile(path);  // 失败时抛异常，new_namespaces 保持完整
      auto& target = new_namespaces[name_space];
      FlatMerge(target, incoming);
    }
    namespaces_ = std::move(new_namespaces);
  }

  // ── 读取接口 ──────────────────────────────────────────────────────────────

  /**
   * @brief 检查一个点分隔的键路径是否存在
   *
   * 用于在读取前做条件判断，避免 GetRequired 抛异常。
   *
   * @param key_path  点分隔路径，如 "armor.light_thresh"
   * @return true 路径存在且不为 YAML::Null
   */
  bool Has(const std::string& key_path) const {
    std::shared_lock<std::shared_mutex> lock(rw_mutex_);
    return Resolve(key_path).IsDefined();
  }

  /**
   * @brief 读取值，键不存在或类型转换失败时返回 default_val
   *
   * 这样调用方不需要 try-catch，适合频繁调用的参数读取。
   * 如果需要发现配置缺失，用 GetRequired 或先 Has 再 Get。
   *
   * @tparam T           目标类型（int / float / double / std::string / bool 等）
   * @param key_path     点分隔路径，如 "armor.light_thresh"
   * @param default_val  键不存在或类型转换失败时的兜底值
   */
  template <typename T>
  T Get(const std::string& key_path, const T& default_val = T{}) const {
    std::shared_lock<std::shared_mutex> lock(rw_mutex_);
    try {
      YAML::Node node = Resolve(key_path);
      if (!node.IsDefined() || node.IsNull()) {
        return default_val;
      }
      return node.as<T>();
    } catch (...) {
      return default_val;
    }
  }

  /**
   * @brief 读取值，键不存在时抛出异常（硬依赖参数用此接口）
   *
   * @tparam T        目标类型
   * @param key_path  点分隔路径
   * @throw std::out_of_range   键不存在
   * @throw std::runtime_error  类型转换失败（如把字符串转 int）
   */
  template <typename T>
  T GetRequired(const std::string& key_path) const {
    std::shared_lock<std::shared_mutex> lock(rw_mutex_);
    YAML::Node node = Resolve(key_path);
    if (!node.IsDefined() || node.IsNull()) {
      throw std::out_of_range("ConfigManager: required key not found: " + key_path);
    }
    try {
      return node.as<T>();
    } catch (const std::exception& err) {
      throw std::runtime_error("ConfigManager: type conversion failed for key '" + key_path +
                               "': " + err.what());
    }
  }

  /**
   * @brief 获取某命名空间下的完整 YAML 节点（只读副本）
   *
   * 返回的是 Clone 副本，外部修改不影响内部存储。
   * 适合把整个子树传给另一个模块自行解析，避免多次 Get 调用。
   * 例如把 "armor" 子树传给装甲板检测器，让它自己取参数。
   *
   * @param ns_path  点分隔路径（留空则返回所有命名空间合并后的整棵树）
   */
  YAML::Node Subtree(const std::string& ns_path = "") const {
    std::shared_lock<std::shared_mutex> lock(rw_mutex_);
    if (ns_path.empty()) {
      // 返回整棵合并视图（克隆一份，防止外部修改）
      return YAML::Clone(MergedView());
    }
    // 先按第一个 '.' 切分：namespace 部分 vs 子路径部分
    auto parts = SplitKey(ns_path);
    // 在合并视图上查找（MergedView 返回临时对象，WalkNode 接受 const&）
    YAML::Node merged = MergedView();
    return YAML::Clone(WalkNode(merged, parts));
  }

  /**
   * @brief 返回所有已加载的文件路径列表（用于调试/诊断）
   */
  std::vector<std::string> LoadedFiles() const {
    std::shared_lock<std::shared_mutex> lock(rw_mutex_);
    std::vector<std::string> paths;
    paths.reserve(loaded_files_.size());
    for (const auto& [path, ignored] : loaded_files_) {
      paths.emplace_back(path);
    }
    return paths;
  }

 private:
  ConfigManager() = default;

  // ── 内部工具函数 ──────────────────────────────────────────────────────────

  /**
   * @brief 将 from 的顶层 key 逐一写入 dest（Map 深度合并，非 Map 直接覆盖）
   */
  static void FlatMerge(YAML::Node& dest, const YAML::Node& from) {
    if (!from.IsMap()) {
      // from 是标量/序列：无法通过 key 遍历，用 YAML::Node 的赋值操作符
      // 注意：dest 是引用，这里赋值的是引用绑定的那个 Node 对象本身——
      // 对于 unordered_map 里存储的 Node，这是合法的
      dest = YAML::Clone(from);
      return;
    }
    for (const auto& item : from) {
      auto key = item.first.as<std::string>();
      if (dest[key] && dest[key].IsMap() && item.second.IsMap()) {
        // 两边都是 Map：递归合并，保留 dest 中已有但 from 中没有的 key
        // dest[key] 返回的是句柄（yaml-cpp Node 是引用计数），
        // 赋给局部变量后用引用传入，写入同样生效到原树
        YAML::Node child = dest[key];
        FlatMergeMap(child, item.second);
      } else {
        dest[key] = item.second;
      }
    }
  }

  /**
   * @brief 递归合并两棵 Map 节点
   *
   * 为什么 dest_map 用 YAML::Node& 引用而不是按值传：
   *   按值传时 clang-tidy 会报 bugprone-easily-swappable-parameters（两参数类型相同）。
   *   改为 YAML::Node&，类型签名变为 (Node&, const Node&)，消除歧义。
   *   yaml-cpp 的 Node 是引用计数句柄，非 const 引用和按值传都指向同一棵树，
   *   写入效果完全一致，改引用不改变任何运行时行为。
   *
   * @param dest_map  目标 Map 节点的非 const 引用（写入通过引用计数生效到原树）
   * @param from_map  源 Map 节点（只读）
   */
  static void FlatMergeMap(YAML::Node& dest_map, const YAML::Node& from_map) {
    for (const auto& item : from_map) {
      auto key = item.first.as<std::string>();
      if (dest_map[key] && dest_map[key].IsMap() && item.second.IsMap()) {
        YAML::Node child = dest_map[key];
        FlatMergeMap(child, item.second);
      } else {
        dest_map[key] = item.second;
      }
    }
  }

  /**
   * @brief 解析 key_path，返回对应的 YAML 节点（找不到时返回 Undefined）
   *
   */
  YAML::Node Resolve(const std::string& key_path) const {
    auto parts = SplitKey(key_path);
    if (parts.empty()) {
      return YAML::Node{YAML::NodeType::Undefined};
    }

    // 阶段1：把第一段当命名空间，查剩余路径
    const auto& first = parts[0];
    auto ns_it = namespaces_.find(first);
    if (ns_it != namespaces_.end() && parts.size() > 1) {
      std::vector<std::string> sub_parts(parts.begin() + 1, parts.end());
      YAML::Node result = WalkNode(YAML::Clone(ns_it->second), sub_parts);
      if (result.IsDefined()) {
        return result;
      }
    }

    // 阶段2：回退到根命名空间（ns=""），用完整路径查找
    auto root_it = namespaces_.find("");
    if (root_it != namespaces_.end()) {
      return WalkNode(root_it->second, parts);
    }

    return YAML::Node{YAML::NodeType::Undefined};
  }

  /**
   * @brief 在一个 Node 上按路径数组逐层步进，返回目标节点
   *
   * 为什么接受 const& 后在内部 Clone：
   *   yaml-cpp 的 operator[] 在 const Node 上返回 Undefined（无法遍历子节点），
   *   必须用非 const 的局部副本来做 key 查找。
   *   接受 const& 让调用方不需要先 Clone，性能和语义都更清晰。
   */
  static YAML::Node WalkNode(const YAML::Node& node, const std::vector<std::string>& parts) {
    YAML::Node cur = YAML::Clone(node);
    for (const auto& part : parts) {
      if (!cur.IsMap()) {
        return YAML::Node{YAML::NodeType::Undefined};
      }
      bool found = false;
      for (const auto& entry : cur) {
        if (entry.first.as<std::string>() == part) {
          cur = entry.second;
          found = true;
          break;
        }
      }
      if (!found) {
        return YAML::Node{YAML::NodeType::Undefined};
      }
    }
    return cur;
  }

  /**
   * @brief 构建所有命名空间合并后的视图（仅供 Subtree() 返回整棵树时使用）
   */
  YAML::Node MergedView() const {
    YAML::Node view;
    // 先把根命名空间的内容铺进去
    auto root_it = namespaces_.find("");
    if (root_it != namespaces_.end() && root_it->second.IsMap()) {
      for (const auto& item : root_it->second) {
        view[item.first.as<std::string>()] = item.second;
      }
    }
    // 再把各命名空间作为顶层 key 铺进去（覆盖根命名空间同名 key）
    for (const auto& [ns, node] : namespaces_) {
      if (!ns.empty()) {
        view[ns] = node;
      }
    }
    return view;
  }

  /**
   * @brief 按 '.' 分割键路径
   */
  static std::vector<std::string> SplitKey(const std::string& key_path) {
    std::vector<std::string> parts;
    std::istringstream stream(key_path);
    std::string token;
    while (std::getline(stream, token, '.')) {
      if (!token.empty()) {
        parts.emplace_back(token);
      }
    }
    return parts;
  }

  // ── 数据成员 ──────────────────────────────────────────────────────────────
  mutable std::shared_mutex rw_mutex_;
  // 方案 A 核心存储：namespace_name → YAML::Node
  // ""（空字符串）表示无命名空间的根配置
  std::unordered_map<std::string, YAML::Node> namespaces_;
  // 用于热重载：记录 {文件路径, 命名空间}
  std::vector<std::pair<std::string, std::string>> loaded_files_;
};

}  // namespace mv

#pragma once

#include <cstddef>
#include <cstdint>

namespace mv::hal::detail::talos_ipc {

// 本文件声明的常量、字段顺序、显式填充和对齐必须与 Daedalus 的 Talos v6
// 共享内存 ABI 完全一致。元数据位于可读写映射，图像像素位于独立的只读三缓冲池。
constexpr std::uint32_t K_SHM_MAGIC = 0x54414C06;  ///< Talos v6 元数据文件标识。
constexpr std::uint32_t K_SHM_VERSION = 6;         ///< 当前支持的共享内存协议版本。
constexpr std::uint8_t K_FLAG_NEW = 0x80;  ///< state 中“存在未消费数据”的标志位。
constexpr std::uint8_t K_INDEX_MASK = 0x03;  ///< state 中可读槽位索引的掩码。
constexpr std::uint8_t K_FORMAT_RGB8 = 0;    ///< 三通道 RGB 字节顺序。
constexpr std::uint8_t K_FORMAT_BGR8 = 1;    ///< 三通道 BGR 字节顺序。
constexpr std::size_t K_BUFFER_COUNT = 3;    ///< 图像、元数据和命令的槽位数量。
constexpr std::size_t K_GROUND_TRUTH_MAX_TARGETS = 16;  ///< 单帧机器人真值容量。
constexpr std::size_t K_GROUND_TRUTH_MAX_ARMORS = 32;   ///< 单帧装甲真值容量。

/** @brief 元数据映射的协议标识、发布时刻及固定图像尺寸。 */
struct alignas(64) ShmHeader {
  std::uint32_t magic;         ///< 固定为 K_SHM_MAGIC。
  std::uint32_t version;       ///< 固定为 K_SHM_VERSION。
  std::uint64_t created_ns;    ///< 发布端创建映射的 Unix epoch 纳秒时间。
  std::uint64_t heartbeat_ns;  ///< 发布端最近一次存活更新的 Unix epoch 纳秒时间。
  std::uint32_t image_width;   ///< 图像池中每帧的宽度，单位为像素。
  std::uint32_t image_height;  ///< 图像池中每帧的高度，单位为像素。
  std::uint8_t pad[32];        ///< ABI 填充，禁止复用。
};

/** @brief Talos ABI 使用的 Hamilton 四元数，字段顺序为 x、y、z、w。 */
struct QuaternionF32 {
  float x;  ///< 虚部 X。
  float y;  ///< 虚部 Y。
  float z;  ///< 虚部 Z。
  float w;  ///< 实部。
};

/**
 * @brief Talos ABI 使用的单精度刚体变换。
 *
 * 命名约定为 parent_t_child，即将 child 坐标变换到 parent；平移单位为米。
 */
struct alignas(32) RigidTransformF32 {
  float translation[3];    ///< parent 坐标系中的 XYZ 平移，单位为米。
  QuaternionF32 rotation;  ///< child 到 parent 的单位四元数旋转。
  std::uint8_t pad[4];     ///< ABI 填充，禁止复用。
};

/** @brief 与图像同帧采样的仿真弹丸累计统计。 */
struct alignas(32) ProjectileStatisticsMeta {
  std::uint64_t timestamp_ns;         ///< 所属采集快照的 Unix epoch 纳秒时间。
  std::uint64_t bullet_launch_count;  ///< 自动或手动生成的 17 mm 弹丸累计数。
  std::uint64_t armor_hit_count;      ///< 弹丸与装甲发生有效碰撞的累计数。
  std::uint32_t rune_hit_count;       ///< 能量机关有效命中累计数。
  std::uint32_t dart_launch_count;    ///< 飞镖发射累计数。
};

/** @brief 视觉端写给 Talos 云台控制器的单条目标命令。 */
struct alignas(32) GimbalCmd {
  std::uint64_t timestamp_ns;  ///< 命令生成的 Unix epoch 纳秒时间。
  float yaw_deg;               ///< 目标偏航角，单位为度。
  float pitch_deg;             ///< 目标俯仰角，单位为度。
  float distance_m;            ///< 目标距离，单位为米。
  std::uint8_t fire_advice;    ///< 非零表示视觉端建议开火。
  std::uint8_t pad[11];        ///< ABI 填充，禁止复用。
};

/** @brief 与单帧图像严格同步的针孔相机内参和 plumb_bob 畸变参数。 */
struct alignas(64) CameraCalibrationMeta {
  std::uint64_t timestamp_ns;  ///< 所属采集快照的 Unix epoch 纳秒时间。
  double fx;                   ///< 水平方向焦距，单位为像素。
  double fy;                   ///< 垂直方向焦距，单位为像素。
  double cx;                   ///< 主点横坐标，单位为像素。
  double cy;                   ///< 主点纵坐标，单位为像素。
  double distortion[5];        ///< 依次为 k1、k2、p1、p2、k3。
  std::uint32_t width;         ///< 标定适用的图像宽度，单位为像素。
  std::uint32_t height;        ///< 标定适用的图像高度，单位为像素。
  std::uint8_t pad[24];        ///< ABI 填充，禁止复用。
};

// 当前 HAL 尚未消费的协议区域仍必须完整占位，以保持与发布端
// #[repr(C, align(...))] 结构的偏移和总尺寸一致。
/** @brief 预留的底盘观测协议区。 */
struct alignas(64) ChassisObservationMeta {
  std::uint8_t bytes[128];
};

/** @brief 仿真器在 world 坐标系中给出的单个机器人真值。 */
struct alignas(32) GroundTruthTargetMeta {
  std::uint64_t frame_sequence;  ///< 所属图像帧序号。
  std::uint64_t timestamp_ns;    ///< 所属采集快照的 Unix epoch 纳秒时间。
  std::uint64_t id;              ///< 本次仿真运行内的稳定目标标识。
  std::uint8_t team;             ///< 队伍编码：0 为红方，1 为蓝方。
  std::uint8_t armor_label;      ///< Talos 装甲类别编码。
  std::uint8_t is_outpost;       ///< 非零表示前哨站等特殊旋转目标。
  std::uint8_t pad1;             ///< ABI 填充，禁止复用。
  float position[3];             ///< 机器人中心 world XYZ 位置，单位为米。
  float yaw_velocity;            ///< 绕 world +Z 的角速度，单位为弧度每秒。
  float yaw;                     ///< 绕 world +Z 的航向角，单位为弧度。
  std::uint8_t pad[16];          ///< ABI 填充，禁止复用。
};

/** @brief 预留的能量机关真值协议区。 */
struct alignas(64) GroundTruthRuneMeta {
  std::uint8_t bytes[128];
};

/** @brief 与图像同帧的单块装甲板几何真值。 */
struct alignas(64) GroundTruthArmorMeta {
  std::uint64_t id;                 ///< 本次仿真运行内的稳定装甲标识。
  std::uint8_t team;                ///< 队伍编码：0 为红方，1 为蓝方。
  std::uint8_t label;               ///< Talos 装甲类别编码。
  std::uint8_t armor_type;          ///< 尺寸编码：0 为小装甲，1 为大装甲。
  std::uint8_t pad1;                ///< ABI 填充，禁止复用。
  float width_m;                    ///< 装甲物理宽度，单位为米。
  float height_m;                   ///< 装甲物理高度，单位为米。
  std::uint8_t pad2[12];            ///< ABI 填充，禁止复用。
  RigidTransformF32 world_t_armor;  ///< armor 到 world 的变换。
  float corners_world[4][3];        ///< world 系角点，顺序为 TL、TR、BR、BL。
  std::uint8_t pad3[16];            ///< ABI 填充，禁止复用。
};

/** @brief 与单帧图像原子发布的一批仿真真值。 */
struct alignas(64) GroundTruthBatchMeta {
  std::uint64_t frame_sequence;  ///< 整批真值所属图像帧序号。
  std::uint64_t timestamp_ns;    ///< 整批真值所属采集快照时间。
  std::uint32_t target_count;    ///< targets 中的有效元素数量。
  std::uint32_t rune_count;      ///< runes 中的有效元素数量；当前 HAL 不消费。
  std::uint32_t armor_count;     ///< armors 中的有效元素数量。
  std::uint32_t pad1;            ///< ABI 填充，禁止复用。
  GroundTruthTargetMeta targets[K_GROUND_TRUTH_MAX_TARGETS];  ///< 机器人真值定长区。
  GroundTruthRuneMeta runes[4];                               ///< 能量机关真值定长区。
  GroundTruthArmorMeta armors[K_GROUND_TRUTH_MAX_ARMORS];     ///< 装甲真值定长区。
};

/** @brief 图像、空间快照和云台执行器状态组成的单帧元数据。 */
struct alignas(64) CapturedFrameMeta {
  std::uint64_t frame_sequence;        ///< 发布端启动后严格递增的帧序号。
  std::uint64_t capture_timestamp_ns;  ///< 整个快照共用的 Unix epoch 纳秒时间。
  std::uint32_t width;                 ///< 当前帧宽度，单位为像素。
  std::uint32_t height;                ///< 当前帧高度，单位为像素。
  std::uint8_t buffer_id;              ///< 图像池中像素所在的槽位索引。
  std::uint8_t format;                 ///< K_FORMAT_RGB8 或 K_FORMAT_BGR8。
  std::uint8_t pad1[6];                ///< ABI 填充，禁止复用。
  std::uint64_t gimbal_consumed_command_timestamp_ns;  ///< 执行器最近消费的命令时间戳。
  float gimbal_yaw_velocity_rad_s;    ///< 实际偏航角速度，单位为弧度每秒。
  float gimbal_pitch_velocity_rad_s;  ///< 实际俯仰角速度，单位为弧度每秒。
  float gimbal_yaw_acceleration_rad_s2;  ///< 实际偏航角加速度，单位为弧度每二次方秒。
  float gimbal_pitch_acceleration_rad_s2;  ///< 实际俯仰角加速度，单位为弧度每二次方秒。
  std::uint8_t gimbal_actuator_mode;               ///< GimbalActuatorMode 的数值编码。
  std::uint8_t gimbal_saturation_flags;            ///< 执行器限位或限速状态位。
  std::uint8_t gimbal_telemetry_valid;             ///< 1 表示本帧云台遥测有效。
  std::uint8_t gimbal_command_valid;               ///< 1 表示最近消费的云台命令有效。
  std::uint8_t pad1_tail[4];                       ///< ABI 填充，禁止复用。
  CameraCalibrationMeta camera_info;               ///< 当前帧对应的相机内参与畸变。
  RigidTransformF32 world_t_gimbal;                ///< gimbal 到 world 的变换。
  RigidTransformF32 gimbal_t_camera_optical;       ///< camera_optical 到 gimbal 的变换。
  RigidTransformF32 gimbal_t_muzzle;               ///< muzzle 到 gimbal 的变换。
  ProjectileStatisticsMeta projectile_statistics;  ///< 同帧弹丸累计统计。
  ChassisObservationMeta chassis_observation;      ///< 同帧底盘观测；当前 HAL 不消费。
  GroundTruthBatchMeta ground_truth;               ///< 同帧仿真真值。
};

/** @brief 发布端写、相机端消费的帧元数据三缓冲。 */
struct alignas(64) FrameTripleBuffer {
  std::uint8_t state;          ///< K_FLAG_NEW 与当前可读槽位索引的原子组合。
  std::uint8_t write_index;    ///< 发布端下一次写入时使用的槽位索引。
  std::uint8_t read_index;     ///< 消费端最近完成读取的槽位索引。
  std::uint8_t pad[61];        ///< 将槽位区对齐到独立缓存行。
  CapturedFrameMeta slots[3];  ///< 固定三个帧元数据槽位。
};

/** @brief 视觉端写、Talos 云台控制器消费的命令三缓冲。 */
struct alignas(64) GimbalTripleBuffer {
  std::uint8_t state;        ///< K_FLAG_NEW 与当前可读槽位索引的原子组合。
  std::uint8_t write_index;  ///< 视觉端下一次写入时使用的槽位索引。
  std::uint8_t read_index;   ///< 控制器最近完成读取的槽位索引。
  std::uint8_t pad[61];      ///< 将槽位区对齐到独立缓存行。
  GimbalCmd slots[3];        ///< 固定三个云台命令槽位。
};

/** @brief Talos 云台执行器最近一次仿真步的完整运行状态。 */
struct alignas(64) RuntimeStateMeta {
  std::uint64_t timestamp_ns;                   ///< 状态采样的 Unix epoch 纳秒时间。
  std::uint64_t consumed_command_timestamp_ns;  ///< 最近消费命令自身携带的时间戳。
  std::uint64_t consumed_at_timestamp_ns;  ///< 控制器消费该命令的 Unix epoch 纳秒时间。
  float target_yaw_rad;                    ///< 当前目标偏航角，单位为弧度。
  float target_pitch_rad;                  ///< 当前目标俯仰角，单位为弧度。
  float actual_yaw_rad;                    ///< 当前实际偏航角，单位为弧度。
  float actual_pitch_rad;                  ///< 当前实际俯仰角，单位为弧度。
  float yaw_velocity_rad_s;                ///< 偏航角速度，单位为弧度每秒。
  float pitch_velocity_rad_s;              ///< 俯仰角速度，单位为弧度每秒。
  float yaw_acceleration_rad_s2;    ///< 偏航角加速度，单位为弧度每二次方秒。
  float pitch_acceleration_rad_s2;  ///< 俯仰角加速度，单位为弧度每二次方秒。
  std::uint8_t following;           ///< 非零表示执行器正在跟随有效目标。
  std::uint8_t actuator_mode;       ///< GimbalActuatorMode 的数值编码。
  std::uint8_t saturation_flags;    ///< 执行器限位或限速状态位。
  std::uint8_t command_valid;       ///< 非零表示当前目标来自有效命令。
  std::uint8_t pad[4];              ///< ABI 填充，禁止复用。
};

/** @brief Talos v6 元数据文件的完整顶层布局。 */
struct alignas(64) ShmMetaRegion {
  ShmHeader header;                ///< 协议头和发布端心跳。
  FrameTripleBuffer frame;         ///< Talos 发布给视觉端的帧快照。
  GimbalTripleBuffer gimbal_cmd;   ///< 视觉端发布给 Talos 的控制命令。
  RuntimeStateMeta runtime_state;  ///< Talos 云台执行器运行状态。
};

// 编译期固定 ABI 尺寸和关键偏移，防止字段或对齐变化静默破坏跨进程协议。
static_assert(sizeof(ShmHeader) == 64);
static_assert(sizeof(QuaternionF32) == 16);
static_assert(sizeof(RigidTransformF32) == 32);
static_assert(sizeof(ProjectileStatisticsMeta) == 32);
static_assert(sizeof(GimbalCmd) == 32);
static_assert(sizeof(CameraCalibrationMeta) == 128);
static_assert(sizeof(GroundTruthTargetMeta) == 64);
static_assert(sizeof(GroundTruthArmorMeta) == 128);
static_assert(sizeof(GroundTruthBatchMeta) == 5696);
static_assert(sizeof(CapturedFrameMeta) == 6144);
static_assert(offsetof(CapturedFrameMeta, projectile_statistics) == 288);
static_assert(offsetof(CapturedFrameMeta, chassis_observation) == 320);
static_assert(offsetof(CapturedFrameMeta, ground_truth) == 448);
static_assert(sizeof(FrameTripleBuffer) == 18496);
static_assert(sizeof(GimbalTripleBuffer) == 192);
static_assert(offsetof(ShmMetaRegion, frame) == 64);
static_assert(offsetof(ShmMetaRegion, gimbal_cmd) == 18560);
static_assert(offsetof(ShmMetaRegion, runtime_state) == 18752);
static_assert(sizeof(ShmMetaRegion) == 18816);

}  // namespace mv::hal::detail::talos_ipc

#ifndef MV_ATTITUDE_WIRE_H
#define MV_ATTITUDE_WIRE_H
/**
 * @brief 版本 1 的共用线协议：整数采用小端序，浮点数采用 IEEE754 binary32，四元数按 wxyz 排列。
 *
 * 帧头依次为 A6 5A、version:u8、type:u8、payload_length:u16、sequence:u32、session:u64。
 * CRC16-CCITT-FALSE 覆盖帧头和载荷，校验值按小端序存储。
 * 云台固件中保留此协议头的副本；传输时逐字段编码，禁止直接发送原生结构体内存。
 */
#include <stdint.h>
#include <string.h>
#define AV_HEADER 18U
#define AV_MAX_FRAME 128U
#define AV_HELLO 1U
#define AV_ACK 2U
#define AV_SYNC 3U
#define AV_SYNC_REPLY 4U
#define AV_ATTITUDE 5U
#define AV_CALIBRATION_SAMPLE 6U
#define AV_GIMBAL_COMMAND 7U
#define AV_ATTITUDE_PAYLOAD 42U
#define AV_CALIBRATION_PAYLOAD 96U
#define AV_GIMBAL_COMMAND_PAYLOAD 12U
#define AV_GIMBAL_YAW_RATE_OFF 0U
#define AV_GIMBAL_PITCH_RATE_OFF 4U
#define AV_GIMBAL_FLAGS_OFF 8U
#define AV_GIMBAL_RESERVED_OFF 10U
#define AV_GIMBAL_FLAG_ENABLE (1U << 0)
#define AV_GIMBAL_FLAG_FIRE_REQUEST (1U << 1)
#define AV_GIMBAL_FLAG_FRICTION_REQUEST (1U << 2)
/* Legacy FIRE implies friction; FRICTION alone never feeds. ENABLE gates both. */
static inline uint16_t AvGimbalCommandFlags(int enabled, int fire, int friction) {
    return enabled ? (uint16_t)(AV_GIMBAL_FLAG_ENABLE |
        (fire ? AV_GIMBAL_FLAG_FIRE_REQUEST : 0U) |
        ((friction || fire) ? AV_GIMBAL_FLAG_FRICTION_REQUEST : 0U)) : 0U;
}
#define AV_CAL_SMALL_DIRECTION_OFF 68U
#define AV_CAL_FLAGS_OFF 69U
#define AV_CAL_GYRO_CAL_OFF 70U
#define AV_CAL_PAYLOAD_VERSION_OFF 71U
#define AV_CAL_PITCH_DIRECTION_OFF 92U
#define AV_CAL_PITCH_FLAGS_OFF 93U
#define AV_CAL_RESERVED_OFF 94U
#define AV_CAL_PAYLOAD_VERSION 2U
#define AV_CAL_REQUIRED_FLAGS 0xf7U
#define AV_CAL_REQUIRED_PITCH_FLAGS 0x1fU
static inline uint64_t AvRead(const uint8_t *p, unsigned n) {
    uint64_t v = 0; unsigned i;
    for (i = 0; i < n; ++i) v |= (uint64_t)p[i] << (8U * i);
    return v;
}
static inline void AvWrite(uint8_t *p, uint64_t v, unsigned n) {
    unsigned i; for (i = 0; i < n; ++i) p[i] = (uint8_t)(v >> (8U * i));
}
static inline float AvFloat(const uint8_t *p) {
    uint32_t bits = (uint32_t)AvRead(p, 4); float v; memcpy(&v, &bits, 4); return v;
}
static inline void AvPutFloat(uint8_t *p, float v) {
    uint32_t bits; memcpy(&bits, &v, 4); AvWrite(p, bits, 4);
}
static inline uint16_t AvCrc(const uint8_t *p, unsigned n) {
    uint16_t crc = 0xffffU; unsigned i, b;
    for (i = 0; i < n; ++i) {
        crc ^= (uint16_t)p[i] << 8;
        for (b = 0; b < 8; ++b)
            crc = (uint16_t)((crc & 0x8000U) ? (crc << 1) ^ 0x1021U : crc << 1);
    }
    return crc;
}
static inline unsigned AvBegin(uint8_t *p, uint8_t type, unsigned payload,
                               uint32_t seq, uint64_t session) {
    p[0] = 0xa6U; p[1] = 0x5aU; p[2] = 1U; p[3] = type;
    AvWrite(p + 4, payload, 2); AvWrite(p + 6, seq, 4); AvWrite(p + 10, session, 8);
    return AV_HEADER + payload + 2U;
}
static inline void AvFinish(uint8_t *p, unsigned size) {
    AvWrite(p + size - 2U, AvCrc(p, size - 2U), 2);
}
#endif

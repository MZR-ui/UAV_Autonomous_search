#ifndef MSP_INTERFACE_MSP_PROTOCOL_H
#define MSP_INTERFACE_MSP_PROTOCOL_H

#include <cstdint>
#include <vector>

namespace msp_interface
{

// ============================================================================
// MSP V2 协议常量
// ============================================================================

/// MSP V2 帧头 (两个字节)
const uint8_t MSP_V2_HEADER[] = {'$', 'X'};

// 消息类型
const uint8_t MSP_V2_TYPE_REQUEST = '<';  // 0x3c
const uint8_t MSP_V2_TYPE_REPLY   = '>';  // 0x3e
const uint8_t MSP_V2_TYPE_ERROR    = '!';  // 0x21

// ============================================================================
// 命令 ID 定义 (需要根据实际飞控固件修改)
// ============================================================================

/// 请求 IMU 传感器数据 (示例值，请查阅飞控源码确认)
const uint16_t MSP_ATTITUDE = 0x006c;

/// 请求气压计高度 (示例值)
const uint16_t MSP2_SENSOR_BARO = 0x1F01;

/// 发送遥控通道数据 (示例值)
const uint16_t MSP2_RC = 0x00C8;

// TODO: 根据您的飞控固件添加更多命令 ID
// 例如 INAV 中的其他传感器命令: MSP2_SENSOR_GPS, MSP2_SENSOR_COMPASS 等

// ============================================================================
// CRC 函数声明
// ============================================================================

/**
 * @brief 计算 DVB-S2 标准的 CRC8 (多项式 0xD5)
 * @param crc 初始 CRC 值
 * @param a   输入字节
 * @return    更新后的 CRC8
 */
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);

/**
 * @brief 计算 CCITT 标准的 CRC16 (多项式 0x1021，初始值 0)
 * @param crc 初始 CRC 值
 * @param a   输入字节
 * @return    更新后的 CRC16
 */
uint16_t crc16_ccitt(uint16_t crc, uint8_t a);

// ============================================================================
// MSP V2 帧打包函数
// ============================================================================

/**
 * @brief 打包 MSP V2 请求帧
 * @param cmd         命令 ID (例如 MSP2_SENSOR_IMU)
 * @param payload     负载数据指针 (可为 nullptr)
 * @param payload_len 负载长度 (字节)
 * @return 包含完整帧的字节向量 (可直接通过串口发送)
 */
std::vector<uint8_t> packMspV2Request(uint16_t cmd, const uint8_t* payload, size_t payload_len);

// ============================================================================
// 可选：传感器数据结构定义 (根据飞控实际数据格式添加)
// ============================================================================
// 示例：IMU 数据结构 (供解析时参考)
// #pragma pack(push, 1)
// struct ImuData {
//     float gyro_x, gyro_y, gyro_z;
//     float acc_x, acc_y, acc_z;
//     float quat_w, quat_x, quat_y, quat_z;
// };
// #pragma pack(pop)

} // namespace msp_interface

#endif // MSP_INTERFACE_MSP_PROTOCOL_H
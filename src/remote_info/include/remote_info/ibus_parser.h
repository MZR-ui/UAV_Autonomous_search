#ifndef REMOTE_INFO_IBUS_PARSER_H
#define REMOTE_INFO_IBUS_PARSER_H

#include <cstdint>
#include <vector>

namespace remote_info
{

// 定义常量（可放在类定义中）
static const uint8_t IBUS_LENGTH = 0x20;
static const uint8_t IBUS_COMMAND40 = 0x40;

/**
 * @brief IBUS 协议解析器
 * 
 * 帧格式：32字节
 * [0] = 0x20 (长度)
 * [1] = 0x40 (命令)
 * [2-29] = 14个通道，每个2字节，小端序
 * [30-31] = 校验和 (0xFFFF - 前30字节和)
 */
class IbusParser
{
public:
    enum State
    {
        WAIT_LENGTH,    // 等待长度字节 0x20
        WAIT_COMMAND,   // 等待命令字节 0x40
        WAIT_DATA,      // 等待数据（30字节，含校验和）
        WAIT_CHECKSUM   // 校验和验证
    };

    IbusParser();
    void reset();
    
    /**
     * @brief 逐字节解析
     * @param byte 输入字节
     * @param out_channels 输出通道数组（成功解析时）
     * @return true 表示解析出一个完整且校验正确的数据包
     */
    bool parseByte(uint8_t byte, std::vector<uint16_t>& out_channels);

private:
    State state_;
    uint8_t buffer_[32];        // 完整帧缓冲区
    uint8_t index_;               // 当前缓冲区索引
    uint16_t checksum_calc_;     // 计算中的校验和
};

} // namespace remote_info

#endif
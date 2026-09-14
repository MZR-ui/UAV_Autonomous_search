#ifndef MSP_INTERFACE_MSP_PARSER_H
#define MSP_INTERFACE_MSP_PARSER_H

#include <cstdint>
#include <vector>

namespace msp_interface
{

class MspV2Parser
{
public:
    enum State
    {
        WAIT_HEADER1,   // 等待 '$'
        WAIT_HEADER2,   // 等待 'X'
        WAIT_TYPE,      // 等待 type
        WAIT_FLAG,      // 等待 flag
        WAIT_CMD1,      // 等待命令低字节
        WAIT_CMD2,      // 等待命令高字节
        WAIT_SIZE1,     // 等待长度低字节
        WAIT_SIZE2,     // 等待长度高字节
        WAIT_PAYLOAD,   // 等待负载数据
        WAIT_CRC        // 等待 CRC8
    };

    MspV2Parser();
    void reset();
    bool parseByte(uint8_t byte, std::vector<uint8_t>& out_packet);

    uint16_t getLastCommand() const { return cmd_record; }  // 获取最近解析完成的命令
private:
    State state_;
    uint8_t type_;          // 消息类型
    uint8_t flag_;          // 标志
    uint16_t cmd_;          // 命令 ID
    uint16_t payload_size_; // 负载长度
    std::vector<uint8_t> payload_;
    uint8_t crc_calc_;      // 计算的 CRC8
    uint8_t crc_received_;  // 接收的 CRC8

    uint16_t cmd_record;          // 命令 ID
};

} // namespace msp_interface

#endif
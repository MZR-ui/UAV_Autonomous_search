#include "remote_info/ibus_parser.h"
#include <ros/ros.h>  // 用于调试打印

namespace remote_info
{

IbusParser::IbusParser()
    : state_(WAIT_LENGTH)
    , index_(0)
    , checksum_calc_(0)
{
}

void IbusParser::reset()
{
    state_ = WAIT_LENGTH;
    index_ = 0;
    // buffer_ 内容无需清空，后续会被覆盖
}

bool IbusParser::parseByte(uint8_t byte, std::vector<uint16_t>& out_channels)
{
    // 逐字节打印（非常详细，调试完后可注释）

    switch (state_)
    {
    case WAIT_LENGTH:
        if (byte == IBUS_LENGTH)
        {
            buffer_[0] = byte;
            index_ = 1;
            state_ = WAIT_COMMAND;
            // printf("WAIT_LENGTH\n");
        }
        break;

    case WAIT_COMMAND:
        if (byte == IBUS_COMMAND40)
        {
            buffer_[1] = byte;
            index_ = 2;
            state_ = WAIT_DATA;
            // printf("WAIT_COMMAND\n");
        }
        else
        {
            // printf("WAIT_COMMAND_warng\n");  // 保留原拼写
            reset();
        }
        break;

    case WAIT_DATA:
        // printf("WAIT_DATA: buffer_[%d]=0x%02x\n", index_, byte);
        buffer_[index_] = byte;
        index_++;
        if (index_ == 32)
        {
            uint16_t checksum_cal = 0xFFFF - buffer_[0] - buffer_[1];
            for (int i = 0; i < 14; i++)
            {
                checksum_cal = checksum_cal - buffer_[2 + i*2] - buffer_[2 + i*2 + 1];
            }
            uint16_t checksum_ibus = buffer_[31] << 8 | buffer_[30];

            if (checksum_cal == checksum_ibus)
            {
                out_channels.clear();
                for (int i = 0; i < 14; i++)
                {
                    uint16_t ch = buffer_[2 + i*2] | (buffer_[2 + i*2 + 1] << 8);
                    out_channels.push_back(ch);
                }
                // printf("success!\r\n");
                reset();
                return true;
            }
            else
            {
                // 原为 ROS_DEBUG_THROTTLE，改为简单打印（每次校验失败都打印，可能很多）
                printf("IBUS checksum mismatch: calc=0x%04x, recv=0x%04x\n",
                       checksum_cal, checksum_ibus);
                reset();
                return false;
            }
        }
        break;

    default:
        reset();
        break;
    }
    return false;
}

} // namespace remote_info
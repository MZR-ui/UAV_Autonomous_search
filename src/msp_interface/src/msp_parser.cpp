#include "msp_interface/msp_parser.h"
#include "msp_interface/msp_protocol.h"
#include <cstdio>  // 用于调试打印（可选）

namespace msp_interface
{

MspV2Parser::MspV2Parser()
    : state_(WAIT_HEADER1)
    , type_(0)
    , flag_(0)
    , cmd_(0)
    , payload_size_(0)
    , crc_calc_(0)
    , crc_received_(0)
{
}

void MspV2Parser::reset()
{
    state_ = WAIT_HEADER1;
    type_ = 0;
    flag_ = 0;
    cmd_ = 0;
    payload_size_ = 0;
    payload_.clear();
    crc_calc_ = 0;
    crc_received_ = 0;
}

bool MspV2Parser::parseByte(uint8_t byte, std::vector<uint8_t>& out_packet)
{
    switch (state_)
    {
    case WAIT_HEADER1:
        if (byte == '$')
            state_ = WAIT_HEADER2;
        break;

    case WAIT_HEADER2:
        if (byte == 'X')
            state_ = WAIT_TYPE;
        else
            reset();
        break;

    case WAIT_TYPE:
        type_ = byte;
        // crc_calc_ = crc8_dvb_s2(0, byte);   // 从 type 开始计算 CRC
        state_ = WAIT_FLAG;
        break;

    case WAIT_FLAG:
        flag_ = byte;
        // crc_calc_ = crc8_dvb_s2(crc_calc_, byte);
        crc_calc_ = crc8_dvb_s2(0, byte);   // 从 type 开始计算 CRC
        state_ = WAIT_CMD1;
        break;

    case WAIT_CMD1:
        cmd_ = byte;
        crc_calc_ = crc8_dvb_s2(crc_calc_, byte);
        state_ = WAIT_CMD2;
        break;

    case WAIT_CMD2:
        cmd_ |= (static_cast<uint16_t>(byte) << 8);
        crc_calc_ = crc8_dvb_s2(crc_calc_, byte);
        state_ = WAIT_SIZE1;
        break;

    case WAIT_SIZE1:
        payload_size_ = byte;
        crc_calc_ = crc8_dvb_s2(crc_calc_, byte);
        state_ = WAIT_SIZE2;
        break;

    case WAIT_SIZE2:
        payload_size_ |= (static_cast<uint16_t>(byte) << 8);
        crc_calc_ = crc8_dvb_s2(crc_calc_, byte);
        if (payload_size_ == 0)
        {
            state_ = WAIT_CRC;
        }
        else
        {
            payload_.clear();
            payload_.reserve(payload_size_);
            state_ = WAIT_PAYLOAD;
        }
        break;

    case WAIT_PAYLOAD:
        payload_.push_back(byte);
        crc_calc_ = crc8_dvb_s2(crc_calc_, byte);
        if (payload_.size() == payload_size_)
            state_ = WAIT_CRC;
        break;

    case WAIT_CRC:
        crc_received_ = byte;
        if (crc_calc_ == crc_received_)
        {
            out_packet = std::move(payload_);
            cmd_record=cmd_;
            reset();
            return true;
        }
        else
        {
            // 调试打印（可选）
            printf("CRC mismatch: calc=0x%02x, recv=0x%02x\n", crc_calc_, crc_received_);
            reset();
            return false;
        }
        break;

    default:
        reset();
        break;
    }
    return false;
}

} // namespace msp_interface
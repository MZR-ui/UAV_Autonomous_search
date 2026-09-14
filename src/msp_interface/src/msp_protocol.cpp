#include <cstdio>  // 在文件开头添加
#include "msp_interface/msp_protocol.h"

namespace msp_interface
{

uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (int i = 0; i < 8; i++)
    {
        if (crc & 0x80)
            crc = (crc << 1) ^ 0xD5;
        else
            crc <<= 1;
    }
    return crc;
}

uint16_t crc16_ccitt(uint16_t crc, uint8_t a)
{
    crc ^= (uint16_t)a << 8;
    for (int i = 0; i < 8; i++)
    {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
    return crc;
}

std::vector<uint8_t> packMspV2Request(uint16_t cmd, const uint8_t* payload, size_t payload_len)
{
    std::vector<uint8_t> frame;
    frame.push_back('$');
    frame.push_back('X');
    frame.push_back('<');
    frame.push_back(0x00);
    frame.push_back(static_cast<uint8_t>(cmd & 0xFF));
    frame.push_back(static_cast<uint8_t>((cmd >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(payload_len & 0xFF));
    frame.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));
    if (payload && payload_len > 0)
        frame.insert(frame.end(), payload, payload + payload_len);

    uint16_t crc = 0;
    for (size_t i = 3; i < payload_len + 8; ++i)
    {
        //printf("CRC calc: i=%zu, byte=0x%02x\n", i, frame[i]);  // 添加打印
        crc = crc8_dvb_s2(crc, frame[i]);
    }

    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    return frame;
}

} // namespace msp_interface
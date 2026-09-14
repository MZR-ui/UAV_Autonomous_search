#include "gimbal_control_serial/gimbal_protocol.hpp"

namespace gimbal_protocol
{
uint16_t CalculateCrc16(uint8_t *ptr, uint8_t len)
{
    uint16_t crc = 0;
    uint8_t da;
    uint16_t crc_ta[16] = {
        0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
        0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    };
    while (len-- != 0)
    {
        da = crc >> 12;
        crc <<= 4;
        crc ^= crc_ta[da ^ (*ptr >> 4)];
        da = crc >> 12;
        crc <<= 4;
        crc ^= crc_ta[da ^ (*ptr & 0x0F)];
        ptr++;
    }
    return crc;
}
}

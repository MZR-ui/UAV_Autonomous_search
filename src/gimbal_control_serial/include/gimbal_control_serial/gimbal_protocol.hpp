#pragma once
#include <cstdint>

namespace gimbal_protocol
{
#pragma pack(push,1)

typedef struct
{
    uint8_t sync[2];
    struct { uint8_t trig:3; uint8_t value:5; } cmd;
    struct { uint8_t:3; int8_t fl_sens:5; } aux;
    struct { uint8_t:3; uint8_t go_zero:1; uint8_t wk_mode:2; uint8_t op_type:2; int16_t op_value; } gbc[3];
    struct { uint8_t:7; uint8_t valid:1; int16_t angle[3]; int16_t accel[3]; } uav;
    struct { uint32_t vert_fov1x:7; uint32_t zoom_value:24; uint32_t reserved:1; float target_angle[2]; } cam;
    uint8_t crc[2];
} Gcu2GbcPkt_t;

typedef struct
{
    uint8_t sync[2];
    uint8_t fw_ver;
    uint8_t hw_err;
    uint8_t inv_flag:1;
    uint8_t gbc_stat:3;
    uint8_t tca_flag:1;
    uint8_t:3;
    struct { uint8_t stat:3; uint8_t value:5; } cmd;
    int16_t cam_rate[3];
    int16_t cam_angle[3];
    int16_t mtr_angle[3];
    uint8_t crc[2];
} Gbc2GcuPkt_t;

#pragma pack(pop)

// CRC16 function declaration
uint16_t CalculateCrc16(uint8_t *ptr, uint8_t len);
}


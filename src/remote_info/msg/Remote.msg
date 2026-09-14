# Remote.msg
# 遥控器通道数据（IBUS协议）

Header header

# 通道数量（IBUS支持最多14通道）
uint8 CHANNEL_COUNT = 14

# 通道值数组，范围 1000-2000
uint16[] channels

# 可选：常用通道别名
# 通常通道映射：1-副翼，2-升降，3-油门，4-方向，5-开关A，6-开关B...
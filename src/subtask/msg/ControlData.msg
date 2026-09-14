# 控制指令消息
# 飞控通道
float32 throttle      # 油门 (0-100% 或 1000-2000)
float32 pitch         # 俯仰角 (度 或 归一化)
float32 roll          # 横滚角 (度)
float32 yaw           # 偏航角 (度)

# 云台控制
uint8   mode_gim      # 云台模式: 0-跟随, 1-锁定, 2-回中...
float32 pitch_gim     # 云台俯仰角 (度)
float32 roll_gim      # 云台横滚角 (度)
float32 yaw_gim       # 云台偏航角 (度)
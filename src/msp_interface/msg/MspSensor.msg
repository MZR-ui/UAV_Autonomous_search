# MspSensor.msg
# 传感器数据消息，对应飞控通过 MSP V2 发布的 IMU、高度等信息

Header header               # 时间戳和坐标系ID（可选）

# 三轴角速度 (单位: rad/s)
float32 gyro_x
float32 gyro_y
float32 gyro_z

# 三轴线加速度 (单位: m/s²)
float32 acc_x
float32 acc_y
float32 acc_z

# 姿态四元数 (归一化)
float32 orientation_x
float32 orientation_y
float32 orientation_z
float32 orientation_w

# 高度 (单位: 米，通常相对于起飞点或海平面)
float32 altitude

# 可选：可根据需要添加更多传感器数据，例如磁力计、GPS 等
# float32 mag_x
# float32 mag_y
# float32 mag_z
# float32 velocity_x
# float32 velocity_y
# float32 velocity_z
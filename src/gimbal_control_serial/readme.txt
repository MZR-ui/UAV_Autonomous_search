# gimbal_control_serial 功能包说明

本功能包主要负责无人机挂载云台（Gimbal）的底层串口通信与控制。它通过指定的私有协议（GCU <-> GBC）将上层 ROS 控制指令打包发送给云台，并实时读取云台返回的位姿和状态数据发布到 ROS 网络中。

## 一、 主要特性
1. **指令下发**：将 ROS 的云台姿态控制指令（Roll/Pitch/Yaw 及工作模式）打包，计算 CRC16 校验后经串口发送。
2. **状态解析**：底层通过独立线程（`readLoop`）异步阻塞读取串口返回的 26 字节数据包（包头 0xB5 0x9A），通过 CRC16 校验后提取温度、角度、角速度、硬件状态等信息。
3. **断线重连**：当串口拔出或打开失败时，不会导致节点崩溃，而是会自动在定时器中尝试重连，连接状态日志以 1Hz 频率降级输出（防止刷屏）。
4. **日志限流**：针对 CRC 校验错误、串口断开等高频异常，使用了 `ROS_WARN_THROTTLE` / `ROS_ERROR_THROTTLE` 控制打印频率（1秒1次）。

## 二、 节点说明
- **节点名称**: `gimbal_control_node`
- **源文件**: `gimbal_control_node.cpp` (ROS 接口与主逻辑), `gimbal_serial.cpp` (底层串口收发与异步读取)

### 2.1 订阅的话题 (Subscribed Topics)
- `/gimbal/cmd` (`gimbal_control_serial/GimbalCmd`)
  接收上层传来的云台控制指令（包含 roll, pitch, yaw 以及模式模式 mode）。

### 2.2 发布的话题 (Published Topics)
- `/gimbal/status` (`gimbal_control_serial/GimbalStatus`)
  发布解析后的云台实时状态（固件版本、硬件报错、相对角度、绝对角度、角速度等）。
- `/gimbal/raw_tx` (`std_msgs/UInt8MultiArray`)
  发布下发给云台的原始协议字节流，主要用于调试和协议抓包分析。

## 三、 自定义消息 (Messages)
- **GimbalCmd.msg**: 云台控制指令。
- **GimbalStatus.msg**: 云台返回状态，包含：
  - `fw_ver` (固件版本)
  - `hw_err` (硬件错误码)
  - `gbc_stat` (云台运行状态)
  - `cam_rate[3]` (相机三轴角速度)
  - `cam_angle[3]` (相机三轴绝对姿态)
  - `mtr_angle[3]` (相机三轴电机相对角度)

## 四、 ROS 参数 (Parameters)
可以在 launch 文件中配置以下参数：
- `~gim_port_name` (string, 默认: "/dev/ttyUSB1")：云台串口设备号。
- `~gim_baud_rate` (int, 默认: 115200)：串口波特率。
- `~gim_publish_rate` (double, 默认: 50.0)：云台控制指令下发频率（Hz）。

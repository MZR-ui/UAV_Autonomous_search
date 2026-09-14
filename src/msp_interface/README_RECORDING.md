# ROS Bag 录制��能使���说明

## 功能��述

已在 `mode_manage_node` 中添加了自��录制���能，可以��过遥控��通道8控制录制��关。

## 录制控制

- **开��录制**: 遥控器通道8的值 < 1350
- **停止录制**: 遥控器通��8的值 >= 1350
- **自动���止**: ���制时长达到10分钟��自动���止

## ��制的���题

系���会录制以��所有 `/msp/` 话题��

- `/msp/imu` - IMU数据
- `/msp/altitude` - 高度���据
- `/msp/gps` - GPS数据
- `/msp/motor` - 电机数据
- `/msp/status` - 状态数据
- `/msp/battery` - 电���数据
- `/msp/esc_telem` - 电���遥测数据
- `/msp/attitude` - 姿态数据
- `/msp/channel` - 通道数��

## 文件管��

- **保存位��**: `/data/rosbag/`
- **命名���则**: `序号_日期时间��.bag`
  - 例如: `001_20260509_143022.bag`
- **��大文��数**: 100���
- **循���覆盖**: 当��件数���到100个时，自��删除��旧的���件

## 配置参数

���以在启动��件中修��以下参��：

```xml
<node pkg="msp_interface" type="mode_manage_node" name="mode_manage_node" output="screen">
    <!-- 录制��关参��� -->
    <param name="bag_save_dir" value="/data/rosbag"/>        <!-- 保存目录 -->
    <param name="max_bag_files" value="100"/>                <!-- 最大文件�� -->
    <param name="max_record_duration" value="600.0"/>        <!-- 最大录制��长(秒) -->
</node>
```

## 使���流程

1. **启动系统**
   ```bash
   roslaunch uav_bringup uav_all.launch
   ```

2. **开始���制**
   - 将���控器通��8拨到低��（< 1350）
   - 系统会��动创���新的bag文件并��始录���
   - 终端会��示: `Started recording to: /data/rosbag/001_20260509_143022.bag`

3. **���止录制**
   - 将遥��器通���8拨��高位��>= 1350）
   - 系统会自��停止���制并关��文件
   - 终端会显��: `Stopped recording`

4. **多��录制**
   - 可以多次开��通道8，每��都会��建新���bag文件
   - ���件序号会��动递增

## 数据转换

### ���法1: 使用提��的Python脚本

将bag文件转换��CSV格式：

```bash
cd /home/mzr/lmw_catkin_ws/uav_ws
python3 src/msp_interface/scripts/bag_to_csv.py /data/rosbag/001_20260509_143022.bag
```

这会创��一个 `001_20260509_143022_csv/` 目录���包含所有��题的CSV文件。

### 方法2: 使用rostopic命令

导出单个��题：

```bash
rostopic echo -b /data/rosbag/001_20260509_143022.bag -p /msp/attitude > attitude.csv
```

### 方法3: 使用PlotJuggler可视��

安���PlotJuggler：
```bash
sudo apt install ros-noetic-plotjuggler-ros
```

��用PlotJuggler打���bag文件：
```bash
rosrun plotjuggler plotjuggler
```

然后���界面中选�� "File" -> "Load rosbag"，选��你的bag文件��

### 方法4: 使��rqt_bag查看

```bash
rqt_bag /data/rosbag/001_20260509_143022.bag
```

## Excel分析

CSV文件可以��接用Excel打开���

1. 打开Excel
2. 选择 "��据" -> "从��本/CSV"
3. 选择生��的CSV文��
4. 可��创建���表、��行数���分析等

## 注意事��

1. **磁盘��间**: 确保 `/data/` 目录有��够的���盘空间
2. **录制���长**: 默��单次���长录制10分钟，��通过���数调整
3. **文���数量**: 默��最多保存100个文��，超���后会自动删除��旧的
4. **权限���题**: 确保程序��权限��� `/data/rosbag/` 目录创建��件

## 故���排查

### ��题1: ��法创建��制目���

**解��方案**:
```bash
sudo mkdir -p /data/rosbag
sudo chown -R $USER:$USER /data/rosbag
```

### ��题2: 录制��有开始

**检��**:
- 查看终端��出是���有错误��息
- 确认��控器通��8的值���否正��（使��� `rostopic echo /remote_order`��
- 确认 `/msp/*` 话题��否在��布数���

### 问题3: bag文件��法打���

**可能原因**:
- 录��过程��程序���常退出，文件��能损���
- 使用 `rosbag info` ���令检查文��完整性

## ��能说明

- 录制���会影响飞控的实时��能
- 使���独立��互斥���保护录制��作
- ���阅队列��小设���为100��避免消息��失

## 示例工��流程

```bash
# 1. 启动��统
roslaunch uav_bringup uav_all.launch

# 2. 飞行��录制���据（��过遥���器通道8���制）

# 3. 查看录制��文件
ls -lh /data/rosbag/

# 4. 转��为CSV
python3 src/msp_interface/scripts/bag_to_csv.py /data/rosbag/001_20260509_143022.bag

# 5. 用Excel打开CSV文��进行���析
# 或使���PlotJuggler��视化
rosrun plotjuggler plotjuggler
```

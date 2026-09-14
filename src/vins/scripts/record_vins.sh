#!/bin/bash
# =====================================================
# 录制 VINS-Fusion 离线测试用 rosbag（视觉 + IMU 原始数据）
#
# 只录 VINS 需要的 3 个话题（color/depth/tf/MSP 都不录，省空间）：
#   /camera/infra1/image_rect_raw   左目  640x480@30fps mono8
#   /camera/infra2/image_rect_raw   右目  640x480@30fps mono8
#   /camera/imu                     IMU  ~200Hz
#
# 用法：
#   ./record_vins.sh              录到 data/rosbag/vins_<时间戳>.bag，Ctrl-C 停止
#   ./record_vins.sh /输出/目录     指定输出目录
#   ./record_vins.sh "" 120       录 120 秒自动停止
#
# 前置：sensing/realsense.launch 已在跑（相机在流数据）
# 数据量：约 18.4 MB/s ≈ 1.1 GB/分钟
# =====================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

OUT_DIR="${1:-$WS_ROOT/data/rosbag}"
DURATION="$2"

mkdir -p "$OUT_DIR"
NAME="vins_$(date +%Y%m%d_%H%M%S).bag"

echo "=============================================="
echo " VINS 离线数据录制"
echo " 输出  : $OUT_DIR/$NAME"
echo " 话题  : infra1 + infra2 + imu"
echo " 数据量: ~18.4 MB/s (~1.1 GB/min)"
[ -n "$DURATION" ] && echo " 时长  : ${DURATION}s"
echo " 停止  : Ctrl-C"
echo "=============================================="

if [ -n "$DURATION" ]; then
    rosbag record -O "$OUT_DIR/$NAME" --duration="$DURATION" \
        /camera/infra1/image_rect_raw \
        /camera/infra2/image_rect_raw \
        /camera/imu
else
    rosbag record -O "$OUT_DIR/$NAME" \
        /camera/infra1/image_rect_raw \
        /camera/infra2/image_rect_raw \
        /camera/imu
fi

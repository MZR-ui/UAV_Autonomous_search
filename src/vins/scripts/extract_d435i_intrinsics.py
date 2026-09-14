#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 RealSense D435i 的 /camera/infra1/camera_info 和 /camera/infra2/camera_info
提取相机内参，生成 VINS-Fusion 需要的 left.yaml / right.yaml。

用法（相机已启动后）:
    source devel/setup.bash
    python3 src/vins/scripts/extract_d435i_intrinsics.py

生成文件:
    src/vins/config/d435i/left.yaml
    src/vins/config/d435i/right.yaml
"""
import rospy
from sensor_msgs.msg import CameraInfo

OUT_LEFT = None  # 运行时填充
OUT_RIGHT = None

left = {}
right = {}


def _on_left(msg):
    left['K'] = msg.K  # fx, 0, cx, 0, fy, cy, 0, 0, 1
    left['w'] = msg.width
    left['h'] = msg.height


def _on_right(msg):
    right['K'] = msg.K
    right['w'] = msg.width
    right['h'] = msg.height


def _write(path, calib):
    k = calib['K']
    fx, fy = k[0], k[4]
    cx, cy = k[2], k[5]
    w, h = calib['w'], calib['h']
    # 图像为 *_image_rect_raw（已去畸变），故畸变参数为 0
    content = f"""%YAML:1.0
---
model_type: PINHOLE
camera_name: camera
image_width: {w}
image_height: {h}
distortion_parameters:
   k1: 0.0
   k2: 0.0
   p1: 0.0
   p2: 0.0
projection_parameters:
   fx: {fx}
   fy: {fy}
   cx: {cx}
   cy: {cy}
"""
    with open(path, 'w') as f:
        f.write(content)
    print(f"[OK] 写入 {path}")
    print(f"     fx={fx} fy={fy} cx={cx} cy={cy}  ({w}x{h})")


def main():
    rospy.init_node('extract_d435i_intrinsics', anonymous=True)

    import os
    pkg_dir = os.path.join(os.path.dirname(__file__), '..')
    global OUT_LEFT, OUT_RIGHT
    OUT_LEFT = os.path.normpath(os.path.join(pkg_dir, 'config/d435i/left.yaml'))
    OUT_RIGHT = os.path.normpath(os.path.join(pkg_dir, 'config/d435i/right.yaml'))

    rospy.Subscriber('/camera/infra1/camera_info', CameraInfo, _on_left, queue_size=1)
    rospy.Subscriber('/camera/infra2/camera_info', CameraInfo, _on_right, queue_size=1)

    rospy.loginfo('等待 /camera/infra1|2/camera_info ...')
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if 'K' in left and 'K' in right:
            _write(OUT_LEFT, left)
            _write(OUT_RIGHT, right)
            rospy.loginfo('内参提取完成，可重新 launch vins_d435i.launch。')
            return
        rate.sleep()

    rospy.logwarn('超时未收到 camera_info，请确认 realsense.launch 已启动且相机已重插。')


if __name__ == '__main__':
    main()

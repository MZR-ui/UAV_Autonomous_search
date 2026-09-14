#!/usr/bin/env python3
"""
实时绘制 VINS-Fusion 的 x-y 轨迹（俯视图）。

比 rqt_plot 好在：
  1) 坐标轴等比（set_aspect equal），轨迹形状不失真
  2) 自己累积全部点，不会滚动消失

用法（先 source devel/setup.bash，且 VINS 在跑）：
    python3 src/vins/scripts/plot_traj_live.py

无头环境（SSH）需 X11 转发：ssh -X
"""
import threading

import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry

_lock = threading.Lock()
xs = []
ys = []


def cb(msg):
    with _lock:
        xs.append(msg.pose.pose.position.x)
        ys.append(msg.pose.pose.position.y)


def main():
    rospy.init_node("traj_plot")
    rospy.Subscriber("/vins_estimator/odometry", Odometry, cb)

    plt.ion()
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("VINS-Fusion x-y trajectory")
    ax.grid(True)
    (line,) = ax.plot([], [], "-o", markersize=2, linewidth=1)
    ax.text(0.02, 0.98, "waiting for /vins_estimator/odometry ...",
            transform=ax.transAxes, va="top", color="gray")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 快照要在锁内取，保证 xs/ys 长度一致（避免回调线程插到一半）
        with _lock:
            if xs:
                line.set_data(list(xs), list(ys))
                ax.relim()
                ax.autoscale_view()
        try:
            fig.canvas.draw()
            fig.canvas.flush_events()
        except Exception:
            pass
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass

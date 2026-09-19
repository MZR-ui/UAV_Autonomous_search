#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace trajectory {

// 位置外环控制器参数（全部可由 bt_config.yaml 配置）
struct CtrlParams
{
    bool   feedback_enabled = true;      // 闭环开关（open_loop / closed_loop）
    bool   lateral_enabled = true;       // 水平闭环开关（false=仅垂直，手动接管让位用）
    double kp_xy = 1.0;                  // 水平位置 P (1/s²)
    double kp_z  = 2.0;                  // 垂直位置 P
    double kd_xy = 0.5;                  // 水平速度 D（无 IMU 速度噪，先温和 0.5；抖就回 0）
    double kd_z  = 0.8;                  // 垂直速度 D（rangefinder/barometer 源 v_M.z 被置 0，此项实际只吃参考速度前馈）
    double gravity = 9.81;               // 重力 (m/s²)
    double angle_max = 0.35;             // ANGLE 满量程倾角 (rad ≈ 20°)，须与 INAV max_angle_inclination 对齐
    double hover_throttle = 1420.0;      // 悬停油门 PWM（实测标定，随电压漂移）
    double k_thr = 30.0;                 // 推力→油门斜率 PWM/(m/s²)（实测标定）
    double pitch_sign = +1.0;            // 俯仰角→PWM 符号（标准 INAV：CH2>1500=低头；正向=低头。首飞前地面验证，可翻转）
    double roll_sign  = +1.0;            // 横滚角→PWM 符号（标准 INAV：CH1>1500=右滚）
    bool   yaw_enabled = false;          // 是否启用航向保持
    double yaw_kp = 0.0;                 // 航向 P（PWM/rad）
    int    pwm_min = 1000;
    int    pwm_max = 2000;
};

// 控制器输出（均为 PWM）
struct CtrlOutput
{
    int throttle_pwm = 1500;
    int roll_pwm  = 1500;
    int pitch_pwm = 1500;
    int yaw_pwm   = 1500;
};

// 位置外环：参考轨迹 + 测量 → 期望姿态/油门 PWM。
// 坐标系：任务系 M（z 向上、x 为机头水平投影、y 向左）。
// 飞控在 ANGLE 模式下把 pitch/roll PWM 当作倾角指令、yaw PWM 当作角速率。
class PositionController
{
public:
    explicit PositionController(const CtrlParams& p) : p_(p) {}

    void update(const Eigen::Vector3d& p_ref, const Eigen::Vector3d& v_ref,
                const Eigen::Vector3d& a_ref,
                const Eigen::Vector3d& p_meas, const Eigen::Vector3d& v_meas,
                double yaw_err, CtrlOutput& out) const
    {
        // 1. 期望加速度（闭环时叠加位置/速度反馈；lateral_enabled=false 时仅垂直反馈）
        Eigen::Vector3d a_des = a_ref;
        if (p_.feedback_enabled) {
            Eigen::Vector3d ep = p_ref - p_meas;
            Eigen::Vector3d ev = v_ref - v_meas;
            if (p_.lateral_enabled) {
                a_des.x() += p_.kp_xy * ep.x() + p_.kd_xy * ev.x();
                a_des.y() += p_.kp_xy * ep.y() + p_.kd_xy * ev.y();
            }
            a_des.z() += p_.kp_z  * ep.z() + p_.kd_z  * ev.z();
        }
        if (!p_.lateral_enabled) {
            a_des.x() = 0.0;  // 仅垂直：水平不产生加速度指令
            a_des.y() = 0.0;
        }

        // 2. 期望比力（含重力补偿）
        Eigen::Vector3d f = a_des;
        f.z() += p_.gravity;

        const double f_norm = f.norm();
        if (f_norm < 1e-6) {
            out = CtrlOutput();
            return;
        }
        Eigen::Vector3d z_b = f / f_norm;

        // 3. 由期望推力方向反解期望 roll/pitch（Z 轴对齐）
        const double pitch = std::atan2(z_b.x(), z_b.z());
        const double roll  = std::atan2(-z_b.y(),
                                        std::sqrt(z_b.x() * z_b.x() + z_b.z() * z_b.z()));

        // 4. 推力 → 油门（近悬停线性化）
        const double throttle = p_.hover_throttle + p_.k_thr * (f_norm - p_.gravity);

        out.throttle_pwm = clampPwm(throttle);
        if (p_.lateral_enabled) {
            out.pitch_pwm = clampPwm(1500.0 + p_.pitch_sign * (pitch / p_.angle_max) * 500.0);
            out.roll_pwm  = clampPwm(1500.0 + p_.roll_sign  * (roll  / p_.angle_max) * 500.0);
        } else {
            out.pitch_pwm = 1500;  // 姿态交还遥控（由 mode_manage override 覆盖）
            out.roll_pwm  = 1500;
        }
        out.yaw_pwm      = p_.yaw_enabled ? clampPwm(1500.0 + p_.yaw_kp * yaw_err) : 1500;
    }

private:
    int clampPwm(double v) const
    {
        return static_cast<int>(std::min<double>(p_.pwm_max,
                              std::max<double>(p_.pwm_min, std::round(v))));
    }

    CtrlParams p_;
};

} // namespace trajectory

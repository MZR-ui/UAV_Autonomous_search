#pragma once

namespace trajectory {

/**
 * @brief 一阶互补速度估计器（单轴）
 *
 * 仿 INAV 的速度环反馈来源：
 *   快通道 = 重力补偿后的线加速度积分（低延迟、干净，但会漂移）
 *   慢通道 = 位置测量差分（去漂移，但噪声大/延迟大）
 *   融合   = v_est = alpha * v_imu + (1-alpha) * v_pos，随后把积分拉回 v_est 防漂移累积
 *
 * 用法：每个 IMU 样本调 integrate(a_lin, dt)；每个位置样本调 correct(v_pos, alpha)。
 */
class ComplementaryVelocityFilter
{
public:
    void reset(double v = 0.0) { v_est_ = v_imu_ = v; }

    // 快通道：积分重力补偿后的线加速度（每个 IMU 样本）
    void integrate(double a_lin, double dt)
    {
        if (dt <= 0.0) return;
        v_imu_ += a_lin * dt;
    }

    // 慢通道：位置差分 + 一阶互补融合，并把积分拉回（防漂移累积）
    void correct(double v_pos, double alpha)
    {
        v_est_ = alpha * v_imu_ + (1.0 - alpha) * v_pos;
        v_imu_ = v_est_;
    }

    double value() const { return v_est_; }

private:
    double v_est_ = 0.0;
    double v_imu_ = 0.0;
};

} // namespace trajectory

#pragma once

#include <string>
#include <vector>

/**
 * @brief 技能文件加载器
 *
 * 文件 A = CSV（时间序列轨迹）
 * 文件 B = YAML（元描述）
 */
class SkillLoader
{
public:
    struct Trajectory
    {
        double interval = 0.02;           // 采样间隔 (s)
        std::vector<double> time;          // t 序列
        std::vector<double> throttle;      // 油门 PWM
        std::vector<double> pitch;         // 俯仰 PWM
        std::vector<double> roll;          // 横滚 PWM
        std::vector<double> yaw;           // 偏航 PWM
        bool empty() const { return time.empty(); }
    };

    struct Meta
    {
        double tolerance_low  = 0.1;      // NORMAL/DEVIATED 边界 (m)
        double tolerance_high = 0.5;      // DEVIATED/DANGER 边界 (m)
        double timeout        = 10.0;     // 超时 (s)
        double target_z       = 1.0;      // 目标高度 (m)
        double max_accel      = 9.8;      // 最大加速度 (m/s²)
    };

    /** 从 CSV 文件加载轨迹 */
    static Trajectory loadTrajectory(const std::string& path);

    /** 从 YAML 文件加载元描述 */
    static Meta loadMeta(const std::string& path);
};

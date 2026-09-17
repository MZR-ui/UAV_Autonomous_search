#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace trajectory {

// 7 阶最小 snap 轨迹（移植自 .doc/0_relevant/quadrotor/traj_planning/traj_opt7.m）
//
// 每段为 7 阶多项式（每轴 8 个系数），约束：
//   - 相邻段在连接点处位置、速度、加速度、jerk、snap、crackle、pop 连续
//   - 起点/终点位置给定，速度/加速度/jerk 归零（静止-静止）
//
// 系数按段排列：第 k 段（0 基）的 8 个系数位于 coeffs_ 的第 [8k, 8k+7] 行，
// 依次对应 t^7, t^6, ..., t^0。每列对应一个轴（x/y/z）。
class MinSnapTrajectory
{
public:
    MinSnapTrajectory() = default;

    // 单段静止-静止：p0 -> p1，历时 duration 秒（最小 snap 的闭式特例）
    static MinSnapTrajectory singleSegment(const Eigen::Vector3d& p0,
                                           const Eigen::Vector3d& p1,
                                           double duration);

    // 多路点：path 为 m+1 个点，ts 为 m+1 个分界时刻（ts[0]=0, ts[m]=总时长）
    static MinSnapTrajectory waypoints(const std::vector<Eigen::Vector3d>& path,
                                       const std::vector<double>& ts);

    // 求 t 时刻的 pos / vel / acc（t 越界自动钳制到 [0, total_time]）
    void evaluate(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel,
                  Eigen::Vector3d& acc) const;

    double totalTime() const { return total_time_; }
    bool   empty() const { return m_ == 0; }

private:
    // 导数阶 r 的多项式行向量（长度 8，对应 [t^7, t^6, ..., t^0] 的系数）
    static Eigen::RowVectorXd polyRow(int r, double t);

    int m_ = 0;                   // 段数
    std::vector<double> ts_;      // 长度 m+1 的分界时刻
    Eigen::MatrixXd coeffs_;      // (8*m) x 3
    double total_time_ = 0.0;
};

inline Eigen::RowVectorXd MinSnapTrajectory::polyRow(int r, double t)
{
    Eigen::RowVectorXd row(8);
    switch (r) {
    case 0: row << std::pow(t,7), std::pow(t,6), std::pow(t,5), std::pow(t,4),
                   std::pow(t,3), std::pow(t,2), t, 1.0; break;
    case 1: row << 7*std::pow(t,6), 6*std::pow(t,5), 5*std::pow(t,4), 4*std::pow(t,3),
                   3*std::pow(t,2), 2*t, 1.0, 0.0; break;
    case 2: row << 42*std::pow(t,5), 30*std::pow(t,4), 20*std::pow(t,3), 12*std::pow(t,2),
                   6*t, 2.0, 0.0, 0.0; break;
    case 3: row << 210*std::pow(t,4), 120*std::pow(t,3), 60*std::pow(t,2), 24*t,
                   6.0, 0.0, 0.0, 0.0; break;
    case 4: row << 840*std::pow(t,3), 360*std::pow(t,2), 120*t, 24.0,
                   0.0, 0.0, 0.0, 0.0; break;
    case 5: row << 2520*std::pow(t,2), 720*t, 120.0, 0.0,
                   0.0, 0.0, 0.0, 0.0; break;
    case 6: row << 5040*t, 720.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0; break;
    default: row.setZero(); break;
    }
    return row;
}

inline MinSnapTrajectory MinSnapTrajectory::singleSegment(const Eigen::Vector3d& p0,
                                                          const Eigen::Vector3d& p1,
                                                          double duration)
{
    std::vector<Eigen::Vector3d> path{p0, p1};
    std::vector<double> ts{0.0, duration};
    return waypoints(path, ts);
}

inline MinSnapTrajectory MinSnapTrajectory::waypoints(const std::vector<Eigen::Vector3d>& path,
                                                      const std::vector<double>& ts)
{
    MinSnapTrajectory traj;
    const int m = static_cast<int>(path.size()) - 1;   // 段数
    if (m < 1 || static_cast<int>(ts.size()) != m + 1) {
        return traj;   // 无效输入 → 空轨迹
    }

    const int n = 8 * m;   // 未知数/约束数
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(n, 3);

    int idx = 0;   // 约束计数器

    // 内部连接点 j = 1 .. m-1，时刻 ts[j]
    for (int j = 1; j <= m - 1; ++j) {
        const double t = ts[j];
        // 位置：相邻两段都等于路点 path[j]
        for (int s = 0; s < 2; ++s) {
            const int seg = j - 1 + s;   // j-1 与 j
            A.block(idx, 8 * seg, 1, 8) = polyRow(0, t);
            Y(idx, 0) = path[j](0); Y(idx, 1) = path[j](1); Y(idx, 2) = path[j](2);
            ++idx;
        }
        // 导数连续（1..6 阶）：seg(j-1) - seg(j) = 0
        for (int r = 1; r <= 6; ++r) {
            A.block(idx, 8 * (j - 1), 1, 8) = polyRow(r, t);
            A.block(idx, 8 * j, 1, 8) = -polyRow(r, t);
            ++idx;
        }
    }

    // 起点（seg 0，t = ts[0]）：位置 + 速度/加速度/jerk = 0
    {
        const int seg = 0;
        const double t = ts[0];
        A.block(idx, 8 * seg, 1, 8) = polyRow(0, t);
        Y(idx, 0) = path[0](0); Y(idx, 1) = path[0](1); Y(idx, 2) = path[0](2); ++idx;
        A.block(idx, 8 * seg, 1, 8) = polyRow(1, t); ++idx;
        A.block(idx, 8 * seg, 1, 8) = polyRow(2, t); ++idx;
        A.block(idx, 8 * seg, 1, 8) = polyRow(3, t); ++idx;
    }

    // 终点（seg m-1，t = ts[m]）：位置 + 速度/加速度/jerk = 0
    {
        const int seg = m - 1;
        const double t = ts[m];
        A.block(idx, 8 * seg, 1, 8) = polyRow(0, t);
        Y(idx, 0) = path[m](0); Y(idx, 1) = path[m](1); Y(idx, 2) = path[m](2); ++idx;
        A.block(idx, 8 * seg, 1, 8) = polyRow(1, t); ++idx;
        A.block(idx, 8 * seg, 1, 8) = polyRow(2, t); ++idx;
        A.block(idx, 8 * seg, 1, 8) = polyRow(3, t); ++idx;
    }

    // 约束总数 = 8(m-1) + 8 = 8m，解 A * coeffs = Y
    traj.coeffs_ = A.colPivHouseholderQr().solve(Y);
    traj.m_ = m;
    traj.ts_ = ts;
    traj.total_time_ = ts[m];
    return traj;
}

inline void MinSnapTrajectory::evaluate(double t, Eigen::Vector3d& pos,
                                        Eigen::Vector3d& vel,
                                        Eigen::Vector3d& acc) const
{
    pos.setZero(); vel.setZero(); acc.setZero();
    if (m_ == 0) return;

    if (t < 0.0) t = 0.0;
    if (t > total_time_) t = total_time_;

    // 找所在段：ts_[k] <= t < ts_[k+1]
    int k = 0;
    for (int i = 1; i < m_; ++i) {
        if (t >= ts_[i]) k = i; else break;
    }

    // 段内求值：x(t) = sum_i c_i * t^(7-i)
    Eigen::Vector3d c_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d c_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d c_acc = Eigen::Vector3d::Zero();
    for (int i = 0; i < 8; ++i) {
        const Eigen::Vector3d c = coeffs_.row(8 * k + i).transpose();
        c_pos += c * std::pow(t, 7 - i);
        if (7 - i >= 1) c_vel += c * (7 - i) * std::pow(t, 6 - i);
        if (7 - i >= 2) c_acc += c * (7 - i) * (6 - i) * std::pow(t, 5 - i);
    }
    pos = c_pos; vel = c_vel; acc = c_acc;
}

} // namespace trajectory

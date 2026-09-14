#include "skill_loader.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

SkillLoader::Trajectory SkillLoader::loadTrajectory(const std::string& path)
{
    Trajectory traj;
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("SkillLoader: cannot open %s", path.c_str());
        return traj;
    }

    std::string line;
    // 跳过标题行
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        double t, thr, pitch, roll, yaw;
        if (iss >> t >> thr >> pitch >> roll >> yaw) {
            traj.time.push_back(t);
            traj.throttle.push_back(thr);
            traj.pitch.push_back(pitch);
            traj.roll.push_back(roll);
            traj.yaw.push_back(yaw);
        }
    }

    if (!traj.time.empty() && traj.time.size() >= 2)
        traj.interval = traj.time[1] - traj.time[0];

    ROS_INFO("SkillLoader: loaded trajectory %s (%zu points, interval=%.3fs)",
             path.c_str(), traj.time.size(), traj.interval);
    return traj;
}

SkillLoader::Meta SkillLoader::loadMeta(const std::string& path)
{
    Meta meta;
    try {
        YAML::Node root = YAML::LoadFile(path);

        if (root["anomaly"]) {
            auto a = root["anomaly"];
            if (a["tolerance_low"])  meta.tolerance_low  = a["tolerance_low"].as<double>();
            if (a["tolerance_high"]) meta.tolerance_high = a["tolerance_high"].as<double>();
        }
        if (root["constraints"]) {
            auto c = root["constraints"];
            if (c["timeout"])          meta.timeout    = c["timeout"].as<double>();
            if (c["max_acceleration"]) meta.max_accel  = c["max_acceleration"].as<double>();
            if (c["completion"] && c["completion"]["position"]) {
                auto p = c["completion"]["position"];
                meta.target_z = p[2].as<double>();
            }
        }

        ROS_INFO("SkillLoader: loaded meta %s (tol_low=%.2f tol_high=%.2f timeout=%.1f target_z=%.2f)",
                 path.c_str(), meta.tolerance_low, meta.tolerance_high,
                 meta.timeout, meta.target_z);
    } catch (const YAML::Exception& e) {
        ROS_ERROR("SkillLoader: YAML parse error in %s: %s", path.c_str(), e.what());
    }
    return meta;
}

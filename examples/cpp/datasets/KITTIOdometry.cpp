#include "KITTIOdometry.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace fs = std::filesystem;

namespace {

std::vector<std::string> GetVelodyneFiles(const fs::path& velodyne_path, int n_scans) {
    std::vector<std::string> velodyne_files;
    for (const auto& entry : fs::directory_iterator(velodyne_path)) {
        if (entry.path().extension() == ".bin") {
            velodyne_files.emplace_back(entry.path().string());
        }
    }
    if (velodyne_files.empty()) {
        std::cerr << velodyne_path << "path doesn't have any .bin" << std::endl;
        exit(1);
    }
    std::sort(velodyne_files.begin(), velodyne_files.end());
    if (n_scans > 0) {
        velodyne_files.erase(velodyne_files.begin() + n_scans, velodyne_files.end());
    }
    return velodyne_files;
}

std::vector<Eigen::Vector3d> ReadKITTIVelodyne(const std::string& path) {
    std::ifstream scan_input(path.c_str(), std::ios::binary);
    assert(scan_input.is_open() && "ReadPointCloud| not able to open file");

    scan_input.seekg(0, std::ios::end);
    uint32_t num_points = scan_input.tellg() / (4 * sizeof(float));
    scan_input.seekg(0, std::ios::beg);

    std::vector<float> values(4 * num_points);
    scan_input.read((char*)&values[0], 4 * num_points * sizeof(float));
    scan_input.close();

    std::vector<Eigen::Vector3d> points;
    points.resize(num_points);
    for (uint32_t i = 0; i < num_points; i++) {
        points[i].x() = values[i * 4];
        points[i].y() = values[i * 4 + 1];
        points[i].z() = values[i * 4 + 2];
    }
    return points;
}

void PreProcessCloud(std::vector<Eigen::Vector3d>& points, YAML::Node cfg) {
    const double min_range = cfg["min_range"].as<float>();
    const double max_range = cfg["max_range"].as<float>();
    points.erase(
        std::remove_if(points.begin(), points.end(), [&](auto p) { return p.norm() > max_range; }),
        points.end());
    points.erase(
        std::remove_if(points.begin(), points.end(), [&](auto p) { return p.norm() < min_range; }),
        points.end());
}

void TransformPoints(std::vector<Eigen::Vector3d>& points, const Eigen::Matrix4d& transformation) {
    for (auto& point : points) {
        Eigen::Vector4d new_point =
            transformation * Eigen::Vector4d(point(0), point(1), point(2), 1.0);
        point = new_point.head<3>() / new_point(3);
    }
}

std::tuple<std::vector<float>, std::vector<Eigen::Matrix4d>> GetGTPoses(
    const fs::path& poses_file, const fs::path& calib_file, const fs::path& timestamps_file) {
    std::vector<float> timestamps;
    std::vector<Eigen::Matrix4d> poses;
    Eigen::Matrix4d T_cam_velo = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d T_velo_cam = Eigen::Matrix4d::Zero();

    // auxiliary variables to read the txt files
    std::string ss;
    float timestamp;
    float P_00, P_01, P_02, P_03, P_10, P_11, P_12, P_13, P_20, P_21, P_22, P_23;

    std::ifstream timestamp_in(timestamps_file, std::ios_base::in);
    while (timestamp_in >> timestamp) {
        timestamps.emplace_back(timestamp);
    }
    std::ifstream calib_in(calib_file, std::ios_base::in);
    // clang-format off
    while (calib_in >> ss >>
           P_00 >> P_01 >> P_02 >> P_03 >>
           P_10 >> P_11 >> P_12 >> P_13 >>
           P_20 >> P_21 >> P_22 >> P_23) {
        if (ss == "Tr:") {
            T_cam_velo << P_00, P_01, P_02, P_03,
                          P_10, P_11, P_12, P_13,
                          P_20, P_21, P_22, P_23,
                          0.00, 0.00, 0.00, 1.00;
            T_velo_cam = T_cam_velo.inverse();
        }
    }
    // clang-format on

    std::ifstream poses_in(poses_file, std::ios_base::in);
    // clang-format off
    while (poses_in >>
           P_00 >> P_01 >> P_02 >> P_03 >>
           P_10 >> P_11 >> P_12 >> P_13 >>
           P_20 >> P_21 >> P_22 >> P_23) {
        Eigen::Matrix4d P;
        P << P_00, P_01, P_02, P_03,
             P_10, P_11, P_12, P_13,
             P_20, P_21, P_22, P_23,
             0.00, 0.00, 0.00, 1.00;
        poses.emplace_back(T_velo_cam * P * T_cam_velo);
    }
    // clang-format on
    return std::make_tuple(timestamps, poses);
}

}  // namespace
namespace datasets {

KITTIDataset::KITTIDataset(const std::string& kitti_root_dir,
                           const std::string& sequence,
                           YAML::Node& cfg,
                           int n_scans)
    : cfg_(cfg),
      preprocess_(cfg["preprocess"].as<bool>()),
      apply_pose_(cfg["apply_pose"].as<bool>()) {
    // TODO: to be completed
    auto kitti_root_dir_ = fs::absolute(fs::path(kitti_root_dir));
    auto kitti_sequence_dir = fs::absolute(fs::path(kitti_root_dir) / "sequences" / sequence);

    // Read data, cache it inside the class.
    std::tie(time_, poses_) =
        GetGTPoses(kitti_root_dir_ / "poses" / std::string(sequence + ".txt"),
                   kitti_sequence_dir / "calib.txt", kitti_sequence_dir / "times.txt");
    scan_files_ = GetVelodyneFiles(fs::absolute(kitti_sequence_dir / "velodyne/"), n_scans);
}

std::tuple<float, std::vector<Eigen::Vector3d>, Eigen::Vector3d> KITTIDataset::operator[](
    int idx) const {
    std::vector<Eigen::Vector3d> points = ReadKITTIVelodyne(scan_files_[idx]);
    if (preprocess_) PreProcessCloud(points, cfg_);
    if (apply_pose_) TransformPoints(points, poses_[idx]);
    const Eigen::Vector3d origin = poses_[idx].block<3, 1>(0, 3);
    const float timestamp = time_[idx];
    return std::make_tuple(timestamp, points, origin);
}

}  // namespace datasets

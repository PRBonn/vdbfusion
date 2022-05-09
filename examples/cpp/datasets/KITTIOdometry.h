#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace datasets {

class KITTIConfig {
public:
    KITTIConfig(const float voxel_size,
                const float sdf_trunc,
                const bool space_carving,
                const std::string& out_dir,
                const bool apply_pose,
                const bool preprocess,
                const bool fill_holes,
                const float min_weight,
                const float min_range,
                const float max_range);

    static KITTIConfig readFromYAML(const std::string& config_path);

public:
    float voxel_size_;
    float sdf_trunc_;
    bool space_carving_;
    std::string out_dir_;
    bool apply_pose_;
    bool preprocess_;
    bool fill_holes_;
    float min_weight_;
    float min_range_;
    float max_range_;
};

class KITTIDataset {
public:
    using Point = Eigen::Vector3d;
    using PointCloud = std::vector<Eigen::Vector3d>;

    explicit KITTIDataset(const std::string& kitti_root_dir,
                          const std::string& sequence,
                          const KITTIConfig& cfg,
                          int n_scans = -1);

    /// Returns a point cloud and the origin of the sensor in world coordinate frames
    [[nodiscard]] std::tuple<PointCloud, Point> operator[](int idx) const;
    [[nodiscard]] std::size_t size() const { return scan_files_.size(); }

private:
    KITTIConfig cfg_;
    std::vector<std::string> scan_files_;
    std::vector<Eigen::Matrix4d> poses_;
};

}  // namespace datasets

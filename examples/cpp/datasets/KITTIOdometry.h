#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace datasets {

class KITTIDataset {
public:
    using Point = Eigen::Vector3d;
    using PointCloud = std::vector<Eigen::Vector3d>;

    explicit KITTIDataset(const std::string& kitti_root_dir,
                          const std::string& sequence,
                          int n_scans = -1);

    explicit KITTIDataset(const std::string& kitti_root_dir,
                          const std::string& sequence,
                          int n_scans = -1,
                          bool apply_pose = true,
                          bool preprocess = true,
                          float min_range = 0.0F,
                          float max_range = std::numeric_limits<float>::max());

    /// Returns a point cloud and the origin of the sensor in world coordinate frames
    [[nodiscard]] std::tuple<PointCloud, Point> operator[](int idx) const;
    [[nodiscard]] std::size_t size() const { return scan_files_.size(); }

public:
    bool apply_pose_ = true;
    bool preprocess_ = true;
    float min_range_ = 0.0F;
    float max_range_ = std::numeric_limits<float>::max();

private:
    std::vector<std::string> scan_files_;
    std::vector<Eigen::Matrix4d> poses_;
};

}  // namespace datasets

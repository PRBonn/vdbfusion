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

    /// Returns a point cloud and the origin of the sensor in world coordinate frames
    [[nodiscard]] std::tuple<PointCloud, Point> operator[](int idx) const;
    [[nodiscard]] std::size_t size() const { return scan_files_.size(); }

private:
    std::vector<std::string> scan_files_;
    std::vector<Eigen::Matrix4d> poses_;
};

}  // namespace datasets

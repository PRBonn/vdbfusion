#pragma once

#include <openvdb/openvdb.h>

#include <Eigen/Core>
#include <functional>
#include <tuple>

namespace vdbfusion {

class VDBVolume {
public:
    VDBVolume(float voxel_size, float sdf_trunc, bool space_carving_ = false);
    ~VDBVolume() = default;

public:
    /// @brief Integrates a new (globally aligned) PointCloud into the current
    /// tsdf_ volume.
    void Integrate(const std::vector<Eigen::Vector3d>& points,
                   const Eigen::Vector3d& origin,
                   const std::function<float(float)>& weighting_function);

    /// @brief Integrates a new (globally aligned) PointCloud into the current
    /// tsdf_ volume.
    void inline Integrate(const std::vector<Eigen::Vector3d>& points,
                          const Eigen::Matrix4d& extrinsics,
                          const std::function<float(float)>& weighting_function) {
        const Eigen::Vector3d& origin = extrinsics.block<3, 1>(0, 3);
        Integrate(points, origin, weighting_function);
    }

    /// @brief Integrate incoming TSDF grid inside the current volume using the TSDF equations
    void Integrate(openvdb::FloatGrid::Ptr grid,
                   const std::function<float(float)>& weighting_function) const;

    /// @brief Integrates a new (globally aligned) PointCloud into the current
    /// tsdf_ volume using Voxblox's "FAST" method.
    void IntegrateFast(const std::vector<Eigen::Vector3d>& points,
                   const Eigen::Vector3d& origin,
                   const std::function<float(float)>& weighting_function);

    /// @brief Fuse a new given sdf value at the given voxel location, thread-safe
    void UpdateTSDF(const float& sdf,
                    const openvdb::Coord& voxel,
                    const std::function<float(float)>& weighting_function) const;

    /// @brief Prune TSDF grids, ideal utility to cleanup a D(x) volume before exporting it
    openvdb::FloatGrid::Ptr Prune(float min_weight) const;

    /// @brief Extracts a TriangleMesh as the iso-surface in the actual volume
    [[nodiscard]] std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>>
    ExtractTriangleMesh(bool fill_holes = true, float min_weight = 0.5) const;

public:
    /// OpenVDB Grids modeling the signed distance field and the weight grid
    openvdb::FloatGrid::Ptr tsdf_;
    openvdb::FloatGrid::Ptr weights_;

    /// VDBVolume public properties
    float voxel_size_;
    float sdf_trunc_;
    bool space_carving_;
};

}  // namespace vdbfusion

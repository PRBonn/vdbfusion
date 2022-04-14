#include "VDBVolume.h"

// OpenVDB
#include <openvdb/Types.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>
#include <unordered_set>

namespace {

float ComputeSDF(const Eigen::Vector3d& origin,
                 const Eigen::Vector3d& point,
                 const Eigen::Vector3d& voxel_center) {
    const Eigen::Vector3d v_voxel_origin = voxel_center - origin;
    const Eigen::Vector3d v_point_voxel = point - voxel_center;
    const double dist = v_point_voxel.norm();
    const double proj = v_voxel_origin.dot(v_point_voxel);
    const double sign = proj / std::abs(proj);
    return static_cast<float>(sign * dist);
}

Eigen::Vector3d GetVoxelCenter(const openvdb::Coord& voxel, const openvdb::math::Transform& xform) {
    const float voxel_size = xform.voxelSize()[0];
    openvdb::math::Vec3d v_wf = xform.indexToWorld(voxel) + voxel_size / 2.0;
    return Eigen::Vector3d(v_wf.x(), v_wf.y(), v_wf.z());
}

}  // namespace

namespace vdbfusion {

VDBVolume::VDBVolume(float voxel_size, float sdf_trunc, bool space_carving /* = false*/)
    : voxel_size_(voxel_size), sdf_trunc_(sdf_trunc), space_carving_(space_carving) {
    tsdf_ = openvdb::FloatGrid::create(sdf_trunc_);
    tsdf_->setName("D(x): signed distance grid");
    tsdf_->setTransform(openvdb::math::Transform::createLinearTransform(voxel_size_));
    tsdf_->setGridClass(openvdb::GRID_LEVEL_SET);

    weights_ = openvdb::FloatGrid::create(0.0f);
    weights_->setName("W(x): weights grid");
    weights_->setTransform(openvdb::math::Transform::createLinearTransform(voxel_size_));
    weights_->setGridClass(openvdb::GRID_UNKNOWN);
}

void VDBVolume::UpdateTSDF(const float& sdf,
                           const openvdb::Coord& voxel,
                           const std::function<float(float)>& weighting_function) const {
    using AccessorRW = openvdb::tree::ValueAccessorRW<openvdb::FloatTree>;
    if (sdf > -sdf_trunc_) {
        AccessorRW tsdf_acc = AccessorRW(tsdf_->tree());
        AccessorRW weights_acc = AccessorRW(weights_->tree());
        const float tsdf = std::min(sdf_trunc_, sdf);
        const float weight = weighting_function(sdf);
        const float last_weight = weights_acc.getValue(voxel);
        const float last_tsdf = tsdf_acc.getValue(voxel);
        const float new_weight = weight + last_weight;
        const float new_tsdf = (last_tsdf * last_weight + tsdf * weight) / (new_weight);
        tsdf_acc.setValue(voxel, new_tsdf);
        weights_acc.setValue(voxel, new_weight);
    }
}

void VDBVolume::Integrate(openvdb::FloatGrid::Ptr grid,
                          const std::function<float(float)>& weighting_function) const {
    for (auto iter = grid->cbeginValueOn(); iter.test(); ++iter) {
        const auto& sdf = iter.getValue();
        const auto& voxel = iter.getCoord();
        this->UpdateTSDF(sdf, voxel, weighting_function);
    }
}

void VDBVolume::Integrate(const std::vector<Eigen::Vector3d>& points,
                          const Eigen::Vector3d& origin,
                          const std::function<float(float)>& weighting_function) {
    if (points.empty()) {
        std::cerr << "PointCloud provided is empty\n";
        return;
    }

    // Get some variables that are common to all rays
    const openvdb::math::Transform& xform = tsdf_->transform();
    const openvdb::Vec3R eye(origin.x(), origin.y(), origin.z());

    // Get the "unsafe" version of the grid accessors
    auto tsdf_acc = tsdf_->getUnsafeAccessor();
    auto weights_acc = weights_->getUnsafeAccessor();

    // Launch a for_each execution, use std::execution::par to parallelize this region
    std::for_each(points.cbegin(), points.cend(), [&](const auto& point) {
        // Get the direction from the sensor origin to the point and normalize it
        const Eigen::Vector3d direction = point - origin;
        openvdb::Vec3R dir(direction.x(), direction.y(), direction.z());
        dir.normalize();

        // Truncate the Ray before and after the source unless space_carving_ is specified.
        const auto depth = static_cast<float>(direction.norm());
        const float t0 = space_carving_ ? 0.0f : depth - sdf_trunc_;
        const float t1 = depth + sdf_trunc_;

        // Create one DDA per ray(per thread), the ray must operate on voxel grid coordinates.
        const auto ray = openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf_);
        openvdb::math::DDA<decltype(ray)> dda(ray);
        do {
            const auto voxel = dda.voxel();
            const auto voxel_center = GetVoxelCenter(voxel, xform);
            const auto sdf = ComputeSDF(origin, point, voxel_center);
            if (sdf > -sdf_trunc_) {
                const float tsdf = std::min(sdf_trunc_, sdf);
                const float weight = weighting_function(sdf);
                const float last_weight = weights_acc.getValue(voxel);
                const float last_tsdf = tsdf_acc.getValue(voxel);
                const float new_weight = weight + last_weight;
                const float new_tsdf = (last_tsdf * last_weight + tsdf * weight) / (new_weight);
                tsdf_acc.setValue(voxel, new_tsdf);
                weights_acc.setValue(voxel, new_weight);
            }
        } while (dda.step());
    });
}

void VDBVolume::IntegrateFast(const std::vector<Eigen::Vector3d>& points,
                          const Eigen::Vector3d& origin,
                          const std::function<float(float)>& weighting_function) {
    if (points.empty()) {
        std::cerr << "PointCloud provided is empty\n";
        return;
    }

    // Get some variables that are common to all rays
    const openvdb::math::Transform& xform = tsdf_->transform();

    // Get the "unsafe" version of the grid accessors
    auto tsdf_acc = tsdf_->getUnsafeAccessor();
    auto weights_acc = weights_->getUnsafeAccessor();

    // Create an unordered set of voxels to track which voxels have been observed
    std::unordered_set<openvdb::Coord> voxel_observed_set_;

    // Launch a for_each execution, use std::execution::par to parallelize this region
    std::for_each(points.cbegin(), points.cend(), [&](const auto& point) {
        const openvdb::Vec3R eye(point.x(), point.y(), point.z());
        // Get the direction from the point to the sensor origin and normalize it
        const Eigen::Vector3d direction = origin - point;
        openvdb::Vec3R dir(direction.x(), direction.y(), direction.z());
        dir.normalize();

        // Truncate the Ray before and after the source unless space_carving_ is specified.
        const auto depth = static_cast<float>(direction.norm());
        const float t0 = -sdf_trunc_;
        const float t1 = space_carving_ ? depth : sdf_trunc_;

        // Ray collision counter
        int consecutive_ray_collisions = 0;
        int max_consecutive_ray_collisions = 2;

        // Create one DDA per ray(per thread), the ray must operate on voxel grid coordinates.
        const auto ray = openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf_);
        openvdb::math::DDA<decltype(ray)> dda(ray);
        do {
            const auto voxel = dda.voxel();
            // Check if the voxel has been observed before, if not, add it to the set else increment the counter
            if (voxel_observed_set_.find(voxel) == voxel_observed_set_.end()) {
                voxel_observed_set_.insert(voxel);
            } else {
                consecutive_ray_collisions++;
            }
            // If the voxel has been observed more than max_consecutive_ray_collisions stop integrating
            if (consecutive_ray_collisions > max_consecutive_ray_collisions) {
                break;
            }
            const auto voxel_center = GetVoxelCenter(voxel, xform);
            const auto sdf = ComputeSDF(origin, point, voxel_center);
            if (sdf > -sdf_trunc_) {
                const float tsdf = std::min(sdf_trunc_, sdf);
                const float weight = weighting_function(sdf);
                const float last_weight = weights_acc.getValue(voxel);
                const float last_tsdf = tsdf_acc.getValue(voxel);
                const float new_weight = weight + last_weight;
                const float new_tsdf = (last_tsdf * last_weight + tsdf * weight) / (new_weight);
                tsdf_acc.setValue(voxel, new_tsdf);
                weights_acc.setValue(voxel, new_weight);
            }
        } while (dda.step());
    });
}

openvdb::FloatGrid::Ptr VDBVolume::Prune(float min_weight) const {
    const auto weights = weights_->tree();
    const auto tsdf = tsdf_->tree();
    const auto background = sdf_trunc_;
    openvdb::FloatGrid::Ptr clean_tsdf = openvdb::FloatGrid::create(sdf_trunc_);
    clean_tsdf->setName("D(x): Pruned signed distance grid");
    clean_tsdf->setTransform(openvdb::math::Transform::createLinearTransform(voxel_size_));
    clean_tsdf->setGridClass(openvdb::GRID_LEVEL_SET);
    clean_tsdf->tree().combine2Extended(tsdf, weights, [=](openvdb::CombineArgs<float>& args) {
        if (args.aIsActive() && args.b() > min_weight) {
            args.setResult(args.a());
            args.setResultIsActive(true);
        } else {
            args.setResult(background);
            args.setResultIsActive(false);
        }
    });
    return clean_tsdf;
}
}  // namespace vdbfusion

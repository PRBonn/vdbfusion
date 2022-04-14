#include "../VDBVolume.h"
#include "gtest/gtest.h"

// OpenVDB
#include <openvdb/openvdb.h>

class VDBVolumeFixture : public ::testing::Test {
protected:
    void SetUp() override { openvdb::initialize(); }
};

TEST_F(VDBVolumeFixture, IntegrateFast) {
    float voxel_size = 0.1;
    float sdf_trunc = 0.3;
    bool space_carving = false;

    vdbfusion::VDBVolume tsdf_volume_simple(voxel_size, sdf_trunc, space_carving);
    vdbfusion::VDBVolume tsdf_volume_fast(voxel_size, sdf_trunc, space_carving);

    const Eigen::Vector3d origin(1.0, 3.0, 2.0);

    // Generate points on a sphere
    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < 1000000; i++) {
        Eigen::Vector3d p;
        p[0] = (float)rand() / RAND_MAX;
        p[1] = (float)rand() / RAND_MAX;
        p[2] = (float)rand() / RAND_MAX;
        p = p.normalized() * 0.5;
        points.emplace_back(p);
    }

    // Integrate the points
    tsdf_volume_simple.Integrate(points, origin, [](float /*unused*/) { return 1.0; });
    tsdf_volume_fast.IntegrateFast(points, origin, [](float /*unused*/) { return 1.0; });

    using AccessorRW = openvdb::tree::ValueAccessorRW<openvdb::FloatTree>;

    AccessorRW tsdf_acc_simple = AccessorRW(tsdf_volume_simple.tsdf_->tree());
    AccessorRW weight_acc_simple = AccessorRW(tsdf_volume_simple.weights_->tree());
    std::vector<float> tsdf_values_simple;
    std::vector<openvdb::Coord> tsdf_coords_simple;
    for (auto it = tsdf_volume_simple.tsdf_->tree().cbeginValueOn(); it; ++it) {
        tsdf_coords_simple.push_back(it.getCoord());
        tsdf_values_simple.push_back(it.getValue());
    }

    AccessorRW tsdf_acc_fast = AccessorRW(tsdf_volume_fast.tsdf_->tree());
    AccessorRW weight_acc_fast = AccessorRW(tsdf_volume_fast.weights_->tree());
    std::vector<float> tsdf_values_fast;
    std::vector<openvdb::Coord> tsdf_coords_fast;
    for (auto it = tsdf_volume_fast.tsdf_->tree().cbeginValueOn(); it; ++it) {
        tsdf_coords_fast.push_back(it.getCoord());
        tsdf_values_fast.push_back(it.getValue());
    }

    ASSERT_EQ(tsdf_values_simple.size(), tsdf_values_fast.size());
    for (size_t i = 0; i < tsdf_values_simple.size(); ++i) {
        ASSERT_EQ(tsdf_coords_simple[i], tsdf_coords_simple[i]);
        ASSERT_NEAR(tsdf_values_simple[i], tsdf_values_fast[i], 0.001);
    }
}

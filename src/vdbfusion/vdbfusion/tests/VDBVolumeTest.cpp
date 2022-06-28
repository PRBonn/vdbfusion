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

    vdbfusion::VDBVolume tsdf_volume(voxel_size, sdf_trunc, space_carving);

    const Eigen::Vector3d origin(1.0, 3.0, 2.0);

    // Generate points on a sphere
    std::vector<Eigen::Vector3d> points;
    std::vector<openvdb::Vec3i> colors;
    for (int i = 0; i < 1000000; i++) {
        Eigen::Vector3d p;
        p[0] = (float)rand() / RAND_MAX;
        p[1] = (float)rand() / RAND_MAX;
        p[2] = (float)rand() / RAND_MAX;
        p = p.normalized() * 0.5;
        points.emplace_back(p);
        openvdb::Vec3i c(0, 100, 255);
        colors.emplace_back(c);
    }

    // Integrate the points
    tsdf_volume.Integrate(points, colors, origin, [](float /*unused*/) { return 1.0; });
}

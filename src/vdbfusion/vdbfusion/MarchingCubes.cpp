#include <openvdb/math/Vec3.h>
#include <openvdb/tree/ValueAccessor.h>

#include <Eigen/Core>
#include <memory>
#include <unordered_map>
#include <vector>

#include "MarchingCubesConst.h"
#include "VDBVolume.h"
#include "Colors.h"

namespace openvdb {
static const openvdb::Coord shift[8] = {
    openvdb::Coord(0, 0, 0), openvdb::Coord(1, 0, 0), openvdb::Coord(1, 1, 0),
    openvdb::Coord(0, 1, 0), openvdb::Coord(0, 0, 1), openvdb::Coord(1, 0, 1),
    openvdb::Coord(1, 1, 1), openvdb::Coord(0, 1, 1),
};
}

// Taken from <open3d/utility/Eigen.h>
namespace {
template <typename T>
struct hash_eigen {
    std::size_t operator()(T const& matrix) const {
        size_t seed = 0;
        for (int i = 0; i < (int)matrix.size(); i++) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};
}  // namespace

namespace vdbfusion {

static constexpr int const X = 0;
static constexpr int const Y = 1;
static constexpr int const Z = 2;
static constexpr int const SOURCE = 0;
static constexpr int const DEST = 1;

template <typename T>
using Accessor = typename openvdb::tree::ValueAccessor<T, true>;

template <typename T, typename Y, typename C>
int GetCubeIndex(const openvdb::Coord& voxel,
                 bool fill_holes,
                 float min_weight,
                 Accessor<T>& weights_acc,
                 Accessor<Y>& tsdf_acc,
                 Accessor<C>& colors_acc,
                 float* vertex_tsdf,
                 openvdb::Vec3f* colors) {
    int cube_index = 0;
    // Iterate through all the 8 neighbour vertices...
    for (int vertex = 0; vertex < 8; vertex++) {
        openvdb::Coord idx = voxel + openvdb::shift[vertex];
        if (!fill_holes) {
            if (weights_acc.getValue(idx) == 0.0f) {
                cube_index = 0;
                break;
            }
        }
        if (weights_acc.getValue(idx) < min_weight) {
            cube_index = 0;
            break;
        }
        vertex_tsdf[vertex] = tsdf_acc.getValue(idx);
        colors[vertex] = colors_acc.getValue(idx);
        if (vertex_tsdf[vertex] < 0.0f) {
            cube_index |= (1 << vertex);
        }
    }
    return cube_index;
}

std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>, std::vector<Eigen::Vector3d>>
VDBVolume::ExtractTriangleMesh(bool fill_holes, float min_weight) const {
    // implementation of marching cubes, based on Open3D
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;
    std::vector<Eigen::Vector3d> colors;

    double half_voxel_length = voxel_size_ * 0.5;
    // Map of "edge_index = (x, y, z, 0) + edge_shift" to "global vertex index"
    std::unordered_map<Eigen::Vector4i, int, hash_eigen<Eigen::Vector4i>, std::equal_to<>,
                       Eigen::aligned_allocator<std::pair<const Eigen::Vector4i, int>>>
        edgeindex_to_vertexindex;
    int edge_to_index[12];

    auto colors_acc = colors_->getAccessor();
    auto tsdf_acc = tsdf_->getAccessor();
    auto weights_acc = weights_->getAccessor();
    for (auto iter = tsdf_->beginValueOn(); iter; ++iter) {
        float vertex_tsdf[8];
        openvdb::Vec3f colors_field[8];
        const openvdb::Coord& voxel = iter.getCoord();
        const int32_t x = voxel.x();
        const int32_t y = voxel.y();
        const int32_t z = voxel.z();
        int cube_index = GetCubeIndex(voxel, fill_holes, min_weight, weights_acc, tsdf_acc,
                                      colors_acc, vertex_tsdf, colors_field);
        if (cube_index == 0 || cube_index == 255) {
            continue;
        }
        // Iterate trough all the edges..
        for (int edge = 0; edge < 12; edge++) {
            if ((edge_table[cube_index] & (1 << edge)) != 0) {
                Eigen::Vector4i edge_index = Eigen::Vector4i(x, y, z, 0) + edge_shift[edge];
                if (edgeindex_to_vertexindex.find(edge_index) == edgeindex_to_vertexindex.end()) {
                    edge_to_index[edge] = (int)vertices.size();
                    edgeindex_to_vertexindex[edge_index] = (int)vertices.size();
                    // set point to source vertex (x, y, z) coordinates
                    Eigen::Vector3d point(half_voxel_length + voxel_size_ * edge_index(X),
                                          half_voxel_length + voxel_size_ * edge_index(Y),
                                          half_voxel_length + voxel_size_ * edge_index(Z));
                    // source vertex TSDF
                    double source_tsdf = std::abs((double)vertex_tsdf[edge_to_vert[edge][SOURCE]]);
                    // destination vertex TSDF
                    double destination_tsdf =
                        std::abs((double)vertex_tsdf[edge_to_vert[edge][DEST]]);
                    // adding delta to reach destination vertex
                    point(edge_index(3)) +=
                        source_tsdf * voxel_size_ / (source_tsdf + destination_tsdf);
                    vertices.push_back(point /* + origin_*/);
                    const auto& source_color = colors_field[edge_to_vert[edge][SOURCE]];
                    const auto& destination_color = colors_field[edge_to_vert[edge][DEST]];

                    openvdb::Vec3f current_color = BlendColors(
                        source_color, destination_tsdf, destination_color, source_tsdf
                    );
                    Eigen::Vector3d color;
                    color[0] = current_color[0];
                    color[1] = current_color[1];
                    color[2] = current_color[2];
                    colors.push_back(color);
                } else {
                    edge_to_index[edge] = edgeindex_to_vertexindex.find(edge_index)->second;
                }
            }
        }
        for (int i = 0; tri_table[cube_index][i] != -1; i += 3) {
            triangles.emplace_back(edge_to_index[tri_table[cube_index][i]],
                                   edge_to_index[tri_table[cube_index][i + 2]],
                                   edge_to_index[tri_table[cube_index][i + 1]]);
        }
    }
    return std::make_tuple(vertices, triangles, colors);
}

}  // namespace vdbfusion

#include <fmt/core.h>
#include <fmt/format.h>
#include <igl/write_triangle_mesh.h>
#include <openvdb/openvdb.h>
#include <vdbfusion/VDBVolume.h>

#include <argparse/argparse.hpp>
#include <filesystem>
#include <string>

#include "datasets/KITTIOdometry.h"
#include "utils/Iterable.h"
#include "utils/Timers.h"

// Namespace aliases
using namespace fmt::literals;
using namespace utils;
namespace fs = std::filesystem;

namespace {

argparse::ArgumentParser ArgParse(int argc, char* argv[]) {
    argparse::ArgumentParser argparser("KITTIPipeline");
    argparser.add_argument("kitti_root_dir").help("The full path to the KITTI dataset");
    argparser.add_argument("--sequence").help("KITTI Sequence");
    argparser.add_argument("--config")
        .help("The full path to the yaml config file")
        .default_value("config/kitti.yaml")
        .action([](const std::string& value) { return value; });
    argparser.add_argument("--n_scans")
        .help("How many scans to map")
        .default_value(int(-1))
        .action([](const std::string& value) { return std::stoi(value); });

    try {
        argparser.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << "Invalid Arguments " << std::endl;
        std::cerr << err.what() << std::endl;
        std::cerr << argparser;
        exit(0);
    }

    auto kitti_root_dir = argparser.get<std::string>("kitti_root_dir");
    if (!fs::exists(kitti_root_dir)) {
        std::cerr << kitti_root_dir << "path doesn't exists" << std::endl;
        exit(1);
    }
    return argparser;
}

}  // namespace

int main(int argc, char* argv[]) {
    auto argparser = ArgParse(argc, argv);

    // VDBVolume configuration
    auto config_path = argparser.get<std::string>("--config");
    datasets::KITTIConfig cfg = datasets::KITTIConfig::readFromYAML(config_path);

    openvdb::initialize();

    // Kitti stuff
    auto n_scans = argparser.get<int>("--n_scans");
    auto kitti_root_dir = fs::path(argparser.get<std::string>("kitti_root_dir"));
    auto sequence = argparser.get<std::string>("--sequence");

    // Initialize dataset
    const auto dataset = datasets::KITTIDataset(kitti_root_dir, sequence, cfg, n_scans);

    fmt::print("Integrating {} scans\n", dataset.size());
    vdbfusion::VDBVolume tsdf_volume(cfg.voxel_size_, cfg.sdf_trunc_, cfg.space_carving_);
    timers::FPSTimer<10> timer;
    for (const auto& [timestamp, scan, origin] : iterable(dataset)) {
        timer.tic();
        tsdf_volume.Integrate(scan, origin, [](float /*unused*/) { return 1.0; });
        timer.toc();
    }

    // Store the grid results to disks
    std::string map_name =
        fmt::format("{out_dir}/kitti_{seq}_{n_scans}_scans", "out_dir"_a = cfg.out_dir_,
                    "seq"_a = sequence, "n_scans"_a = n_scans);
    {
        timers::ScopeTimer timer("Writing VDB grid to disk");
        auto tsdf_grid = tsdf_volume.tsdf_;
        std::string filename = fmt::format("{map_name}.vdb", "map_name"_a = map_name);
        openvdb::io::File(filename).write({tsdf_grid});
    }

    // Run marching cubes and save a .ply file
    {
        timers::ScopeTimer timer("Writing Mesh to disk");
        auto [vertices, triangles] =
            tsdf_volume.ExtractTriangleMesh(cfg.fill_holes_, cfg.min_weight_);

        // TODO: Fix this!
        Eigen::MatrixXd V(vertices.size(), 3);
        for (size_t i = 0; i < vertices.size(); i++) {
            V.row(i) = Eigen::VectorXd::Map(&vertices[i][0], vertices[i].size());
        }

        // TODO: Also this
        Eigen::MatrixXi F(triangles.size(), 3);
        for (size_t i = 0; i < triangles.size(); i++) {
            F.row(i) = Eigen::VectorXi::Map(&triangles[i][0], triangles[i].size());
        }
        std::string filename = fmt::format("{map_name}.ply", "map_name"_a = map_name);
        igl::write_triangle_mesh(filename, V, F, igl::FileEncoding::Binary);
    }

    return 0;
}

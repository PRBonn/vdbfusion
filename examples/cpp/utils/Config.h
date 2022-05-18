#include <filesystem>
#include <fstream>
#include <string>

#include "yaml-cpp/yaml.h"

namespace vdbfusion {
struct VDBFusionConfig {
    float voxel_size_;
    float sdf_trunc_;
    bool space_carving_;
    float min_weight_;
    bool fill_holes_;

    static inline VDBFusionConfig LoadFromYAML(const std::string& path) {
        std::ifstream config_file(path, std::ios_base::in);
        auto config = YAML::Load(config_file);

        return VDBFusionConfig{config["voxel_size"].as<float>(),    //
                               config["sdf_trunc"].as<float>(),     //
                               config["space_carving"].as<bool>(),  //
                               config["min_weight"].as<float>(),    //
                               config["fill_holes"].as<bool>()};
    }
};
}  // namespace vdbfusion

namespace datasets {
struct KITTIConfig {
    bool apply_pose_;
    bool preprocess_;
    float min_range_;
    float max_range_;

    static inline KITTIConfig LoadFromYAML(const std::string& path) {
        std::ifstream config_file(path, std::ios_base::in);
        auto config = YAML::Load(config_file);

        return KITTIConfig{config["apply_pose"].as<bool>(),  //
                           config["preprocess"].as<bool>(),  //
                           config["min_range"].as<float>(),  //
                           config["max_range"].as<float>()};
    }
};
}  // namespace datasets
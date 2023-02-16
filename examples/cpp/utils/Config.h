/*
 * MIT License
 *
 * # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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
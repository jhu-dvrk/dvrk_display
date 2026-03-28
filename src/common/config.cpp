#include "config.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>

namespace sv {

bool Config::load_from_file(const std::string& path, Json::Value& root) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open JSON: " << path << std::endl;
        return false;
    }

    try {
        ifs >> root;
    } catch (const std::exception& e) {
        std::cerr << "JSON parse error in " << path << ": " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool Config::check_type(const Json::Value& root, const std::string& expected_type, const std::string& path) {
    if (!root.isMember("type")) {
        std::cerr << "Error: JSON file '" << path << "' is missing the \"type\" field. "
                  << "Expected \"" << expected_type << "\"." << std::endl;
        return false;
    }

    const std::string actual_type = root["type"].asString();
    if (actual_type == expected_type) {
        return true;
    }

    static const std::map<std::string, std::string> type_map = {
        {"stereo_viewer_config_v1", "sv::config@1.0.0"}
    };

    const auto it = type_map.find(expected_type);
    if (it != type_map.end() && actual_type == it->second) {
        return true;
    }

    std::cerr << "Error: Incompatible JSON type in '" << path << "'. "
              << "Found \"" << actual_type << "\", but expected \"" << expected_type << "\"."
              << std::endl;
    return false;
}

StereoConfig Config::parse_stereo_config(const Json::Value& root) {
    StereoConfig cfg;

    const Json::Value& stereo = root.isMember("stereo") ? root["stereo"] : root;
    if (stereo.isMember("name")) {
        cfg.name = stereo["name"].asString();
    }
    if (cfg.name.empty()) {
        cfg.name = "stereo_preview";
    }
    if (stereo.isMember("original_width")) {
        cfg.original_width = stereo["original_width"].asInt();
    }
    if (stereo.isMember("original_height")) {
        cfg.original_height = stereo["original_height"].asInt();
    }
    if (stereo.isMember("crop_width")) {
        cfg.crop_width = stereo["crop_width"].asInt();
    }
    if (stereo.isMember("crop_height")) {
        cfg.crop_height = stereo["crop_height"].asInt();
    }
    if (stereo.isMember("horizontal_shift_px")) {
        cfg.horizontal_shift_px = stereo["horizontal_shift_px"].asInt();
    }
    if (stereo.isMember("vertical_shift_px")) {
        cfg.vertical_shift_px = stereo["vertical_shift_px"].asInt();
    }
    if (stereo.isMember("sink")) {
        cfg.sink = stereo["sink"].asString();
    }
    if (stereo.isMember("unixfd_socket_path")) {
        cfg.unixfd_socket_path = stereo["unixfd_socket_path"].asString();
        cfg.has_unixfd_socket_path = !cfg.unixfd_socket_path.empty();
    }
    if (stereo.isMember("estimated_latency")) {
        cfg.estimated_latency = stereo["estimated_latency"].asDouble();
    }

    if (stereo.isMember("left") && stereo["left"].isObject()) {
        const auto& left = stereo["left"];
        if (left.isMember("source")) {
            cfg.left.source = left["source"].asString();
        }
        if (left.isMember("original_width")) {
            cfg.left.original_width = left["original_width"].asInt();
        }
        if (left.isMember("original_height")) {
            cfg.left.original_height = left["original_height"].asInt();
        }
    }

    if (stereo.isMember("left_stream")) {
        cfg.left.source = stereo["left_stream"].asString();
    }

    if (stereo.isMember("right") && stereo["right"].isObject()) {
        const auto& right = stereo["right"];
        if (right.isMember("source")) {
            cfg.right.source = right["source"].asString();
        }
        if (right.isMember("original_width")) {
            cfg.right.original_width = right["original_width"].asInt();
        }
        if (right.isMember("original_height")) {
            cfg.right.original_height = right["original_height"].asInt();
        }
    }

    if (stereo.isMember("right_stream")) {
        cfg.right.source = stereo["right_stream"].asString();
    }

    if (root.isMember("left_stream")) {
        cfg.left.source = root["left_stream"].asString();
    }
    if (root.isMember("right_stream")) {
        cfg.right.source = root["right_stream"].asString();
    }

    if (cfg.original_width <= 0) {
        if (cfg.left.original_width > 0 && cfg.right.original_width > 0) {
            cfg.original_width = std::min(cfg.left.original_width, cfg.right.original_width);
        } else {
            cfg.original_width = std::max(cfg.left.original_width, cfg.right.original_width);
        }
    }
    if (cfg.original_height <= 0) {
        if (cfg.left.original_height > 0 && cfg.right.original_height > 0) {
            cfg.original_height = std::min(cfg.left.original_height, cfg.right.original_height);
        } else {
            cfg.original_height = std::max(cfg.left.original_height, cfg.right.original_height);
        }
    }
    if (cfg.crop_width <= 0) {
        cfg.crop_width = cfg.original_width;
    }
    if (cfg.crop_height <= 0) {
        cfg.crop_height = cfg.original_height;
    }

    return cfg;
}

AppConfig Config::parse_app_config(const Json::Value& root) {
    AppConfig cfg;
    cfg.viewer_name = root.get("viewer_name", "dvrk_stereo_viewer").asString();
    cfg.console = root.get("console", "console").asString();
    if (cfg.console.empty()) {
        cfg.console = "console";
    }
    cfg.stereo = parse_stereo_config(root);

    return cfg;
}

}  // namespace sv

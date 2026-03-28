#include "config.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>

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
    if (actual_type != expected_type) {
        std::cerr << "Error: Incompatible JSON type in '" << path << "'. "
                  << "Found \"" << actual_type << "\", but expected \"" << expected_type << "\"."
                  << std::endl;
        return false;
    }
    return true;
}

AppConfig Config::parse_app_config(const Json::Value& root) {
    AppConfig cfg;
    cfg.name = root.get("name", "dvrk_stereo_viewer").asString();
    if (cfg.name.empty()) {
        cfg.name = "dvrk_stereo_viewer";
    }
    cfg.dvrk_console_namespace = root.get("dvrk_console_namespace", "console").asString();
    if (cfg.dvrk_console_namespace.empty()) {
        cfg.dvrk_console_namespace = "console";
    }
    if (root.isMember("ros_image_publishers") && root["ros_image_publishers"].isArray()) {
        for (const auto& item : root["ros_image_publishers"]) {
            if (item.isString()) {
                cfg.ros_image_publishers.push_back(item.asString());
            }
        }
    }
    cfg.overlay_alpha = root.get("overlay_alpha", 0.7).asDouble();

    if (root.isMember("original_width")) {
        cfg.original_width = root["original_width"].asInt();
    }
    if (root.isMember("original_height")) {
        cfg.original_height = root["original_height"].asInt();
    }
    if (root.isMember("crop_width")) {
        cfg.crop_width = root["crop_width"].asInt();
    }
    if (root.isMember("crop_height")) {
        cfg.crop_height = root["crop_height"].asInt();
    }
    if (root.isMember("horizontal_shift_px")) {
        cfg.horizontal_shift_px = root["horizontal_shift_px"].asInt();
    }
    if (root.isMember("vertical_shift_px")) {
        cfg.vertical_shift_px = root["vertical_shift_px"].asInt();
    }
    {
        const std::string sink_type = root.get("sink", "glimage").asString();
        if (sink_type == "custom") {
            cfg.sink_stream = root.get("sink_stream", "").asString();
        } else {
            // default: "glimage"
            cfg.sink_stream = "glimagesink sync=false force-aspect-ratio=false";
        }
    }
    if (root.isMember("unixfd_socket_path")) {
        cfg.unixfd_socket_path = root["unixfd_socket_path"].asString();
        cfg.has_unixfd_socket_path = !cfg.unixfd_socket_path.empty();
    }
    if (root.isMember("left_stream")) {
        cfg.left.source = root["left_stream"].asString();
    }
    if (root.isMember("right_stream")) {
        cfg.right.source = root["right_stream"].asString();
    }

    if (cfg.crop_width <= 0) {
        cfg.crop_width = cfg.original_width;
    }
    if (cfg.crop_height <= 0) {
        cfg.crop_height = cfg.original_height;
    }

    return cfg;
}

}  // namespace sv

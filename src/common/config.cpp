#include "config.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>

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

    auto parse_color = [](const Json::Value& node) {
        ColorAdjustment color;
        if (node.isMember("brightness")) color.brightness = node["brightness"].asDouble();
        if (node.isMember("contrast")) color.contrast = node["contrast"].asDouble();
        if (node.isMember("saturation")) color.saturation = node["saturation"].asDouble();
        if (node.isMember("hue")) color.hue = node["hue"].asDouble();
        return color;
    };

    if (root.isMember("unixfd_socket_path") && root["unixfd_socket_path"].isString()) {
        cfg.unixfd_socket_path = root["unixfd_socket_path"].asString();
    }

    if (root.isMember("stream") && root["stream"].isString()) {
        cfg.stream = root["stream"].asString();
    }

    if (root.isMember("left_color")) {
        cfg.left_color = parse_color(root["left_color"]);
    }
    if (root.isMember("right_color")) {
        cfg.right_color = parse_color(root["right_color"]);
    }

    cfg.name = root.get("name", "dvrk_display").asString();
    if (cfg.name.empty()) {
        cfg.name = "dvrk_display";
    }
    cfg.dvrk_console_namespace = root.get("dvrk_console_namespace", "console").asString();
    if (cfg.dvrk_console_namespace.empty()) {
        cfg.dvrk_console_namespace = "console";
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
    if (root.isMember("display_horizontal_offset_px")) {
        cfg.display_horizontal_offset_px = root["display_horizontal_offset_px"].asInt();
    }
    cfg.preserve_size = root.get("preserve_size", true).asBool();
    if (root.isMember("sinks") && root["sinks"].isArray()) {
        for (const auto& item : root["sinks"]) {
            if (!item.isString()) {
                continue;
            }

            const std::string sink_type = item.asString();
            cfg.sinks.push_back(sink_type);
            if (sink_type == "glimage") {
                cfg.sink_streams.push_back("glimagesink sync=false force-aspect-ratio=false");
            } else if (sink_type == "glimages") {
                cfg.sink_streams.push_back("glimagesink sync=false force-aspect-ratio=false");
                cfg.sink_streams.push_back("glimagesink sync=false force-aspect-ratio=false");
            }
        }
    }
    if (root.isMember("unixfdsinks") && root["unixfdsinks"].isArray()) {
        for (const auto& item : root["unixfdsinks"]) {
            if (!item.isMember("stream")) {
                continue;
            }
            UnixfdSinkConfig sc;
            sc.stream = item["stream"].asString();
            sc.name = item.get("name", "").asString();
            sc.socket_path = item.get("socket_path", "").asString();
            cfg.unixfd_sinks.push_back(sc);
        }
    }
    std::string config_type = root.get("type", "").asString();
    
    if (config_type == "dd::mono_config@1.0.0") {
        if (cfg.stream.empty()) {
            throw std::runtime_error("Configuration error: Required field 'stream' is missing for mono config.");
        }
    } else {
        if (root.isMember("left")) {
            cfg.left.source = root["left"].asString();
            if (cfg.left.source.empty()) {
                throw std::runtime_error("Configuration error: 'left' is present but empty.");
            }
        } else {
            throw std::runtime_error("Configuration error: Required field 'left' is missing.");
        }

        if (root.isMember("right")) {
            cfg.right.source = root["right"].asString();
            if (cfg.right.source.empty()) {
                throw std::runtime_error("Configuration error: 'right' is present but empty.");
            }
        } else {
            throw std::runtime_error("Configuration error: Required field 'right' is missing.");
        }
    }

    if (cfg.crop_width <= 0) {
        cfg.crop_width = cfg.original_width;
    }
    if (cfg.crop_height <= 0) {
        cfg.crop_height = cfg.original_height;
    }

    if (root.isMember("extra_streams") && root["extra_streams"].isObject()) {
        const Json::Value& es = root["extra_streams"];
        if (es.isMember("monos") && es["monos"].isArray()) {
            for (const auto& item : es["monos"]) {
                if (!item.isString()) continue;
                if (!item.asString().empty()) {
                    cfg.extra_streams.monos.push_back(item.asString());
                }
            }
        }
        
        if (es.isMember("stereos") && es["stereos"].isArray()) {
            for (const auto& item : es["stereos"]) {
                if (!item.isObject()) continue;
                if (item.isMember("left") && item.isMember("right") && 
                    item["left"].isString() && item["right"].isString()) {
                    StereoExtraStream ses;
                    ses.left = item["left"].asString();
                    ses.right = item["right"].asString();
                    if (!ses.left.empty() && !ses.right.empty()) {
                        cfg.extra_streams.stereos.push_back(ses);
                    }
                }
            }
        }

        // Enforce maximum of 2 total streams (monos + stereos)
        int total_streams = cfg.extra_streams.monos.size() + cfg.extra_streams.stereos.size();
        if (total_streams > 2) {
            std::cerr << "Warning: Maximum of 2 extra streams allowed. Truncating." << std::endl;
            while (cfg.extra_streams.monos.size() + cfg.extra_streams.stereos.size() > 2) {
                if (!cfg.extra_streams.stereos.empty()) {
                    cfg.extra_streams.stereos.pop_back();
                } else {
                    cfg.extra_streams.monos.pop_back();
                }
            }
        }
        if (es.isMember("scale")) {
            const double s = es["scale"].asDouble();
            cfg.extra_streams.scale = std::max(0.01, std::min(0.99, s));
        }
    }

    return cfg;
}

}  // namespace sv

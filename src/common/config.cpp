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
        if (node.isMember("contrast"))   color.contrast   = node["contrast"].asDouble();
        if (node.isMember("saturation")) color.saturation = node["saturation"].asDouble();
        if (node.isMember("hue"))        color.hue        = node["hue"].asDouble();
        return color;
    };

    // ── Root-level fields (common to stereo and mono) ─────────────────────────
    cfg.name = root.get("name", "dvrk_display").asString();
    if (cfg.name.empty()) cfg.name = "dvrk_display";

    cfg.dvrk_console_namespace = root.get("dvrk_console_namespace", "console").asString();
    if (cfg.dvrk_console_namespace.empty()) cfg.dvrk_console_namespace = "console";

    cfg.overlay_alpha = root.get("overlay_alpha", 0.7).asDouble();
    cfg.preserve_size = root.get("preserve_size", true).asBool();

    if (root.isMember("display_horizontal_offset_px"))
        cfg.display_horizontal_offset_px = root["display_horizontal_offset_px"].asInt();

    if (root.isMember("unixfd_socket_path") && root["unixfd_socket_path"].isString())
        cfg.unixfd_socket_path = root["unixfd_socket_path"].asString();

    if (root.isMember("sinks") && root["sinks"].isArray()) {
        for (const auto& item : root["sinks"]) {
            if (!item.isString()) continue;
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
            if (!item.isMember("stream")) continue;
            UnixfdSinkConfig sc;
            sc.stream      = item["stream"].asString();
            sc.name        = item.get("name", "").asString();
            sc.socket_path = item.get("socket_path", "").asString();
            cfg.unixfd_sinks.push_back(sc);
        }
    }

    // ── camera object (required) ──────────────────────────────────────────────
    if (!root.isMember("camera") || !root["camera"].isObject())
        throw std::runtime_error("Configuration error: Required 'camera' object is missing.");

    const Json::Value& cam = root["camera"];
    const std::string config_type = root.get("type", "").asString();

    if (config_type == "dd::mono_config@1.0.0") {
        // Mono: camera.stream (required), camera.size (optional), camera.color (optional)
        if (!cam.isMember("stream") || !cam["stream"].isString() || cam["stream"].asString().empty())
            throw std::runtime_error("Configuration error: Required field 'camera.stream' is missing or empty.");
        cfg.stream = cam["stream"].asString();

        if (cam.isMember("size") && cam["size"].isObject()) {
            const Json::Value& sz = cam["size"];
            if (sz.isMember("width"))  cfg.original_width  = sz["width"].asInt();
            if (sz.isMember("height")) cfg.original_height = sz["height"].asInt();
        }

        if (cam.isMember("color") && cam["color"].isObject())
            cfg.left_color = parse_color(cam["color"]);

    } else {
        // Stereo: camera.size (required), camera.left/right (required),
        //         camera.crop (optional), camera.alignment (optional)

        if (!cam.isMember("size") || !cam["size"].isObject())
            throw std::runtime_error("Configuration error: Required field 'camera.size' is missing.");
        const Json::Value& sz = cam["size"];
        if (!sz.isMember("width") || !sz.isMember("height"))
            throw std::runtime_error("Configuration error: 'camera.size' must define both 'width' and 'height'.");
        cfg.original_width  = sz["width"].asInt();
        cfg.original_height = sz["height"].asInt();

        if (!cam.isMember("left") || !cam["left"].isObject())
            throw std::runtime_error("Configuration error: Required field 'camera.left' is missing.");
        const Json::Value& lft = cam["left"];
        if (!lft.isMember("stream") || lft["stream"].asString().empty())
            throw std::runtime_error("Configuration error: Required field 'camera.left.stream' is missing or empty.");
        cfg.left.source = lft["stream"].asString();
        if (lft.isMember("color") && lft["color"].isObject())
            cfg.left_color = parse_color(lft["color"]);

        if (!cam.isMember("right") || !cam["right"].isObject())
            throw std::runtime_error("Configuration error: Required field 'camera.right' is missing.");
        const Json::Value& rgt = cam["right"];
        if (!rgt.isMember("stream") || rgt["stream"].asString().empty())
            throw std::runtime_error("Configuration error: Required field 'camera.right.stream' is missing or empty.");
        cfg.right.source = rgt["stream"].asString();
        if (rgt.isMember("color") && rgt["color"].isObject())
            cfg.right_color = parse_color(rgt["color"]);

        if (cam.isMember("crop") && cam["crop"].isObject()) {
            const Json::Value& crop = cam["crop"];
            if (crop.isMember("width"))  cfg.crop_width  = crop["width"].asInt();
            if (crop.isMember("height")) cfg.crop_height = crop["height"].asInt();
        }

        if (cam.isMember("alignment") && cam["alignment"].isObject()) {
            const Json::Value& align = cam["alignment"];
            if (align.isMember("horizontal_shift_px"))
                cfg.horizontal_shift_px = align["horizontal_shift_px"].asInt();
            if (align.isMember("vertical_shift_px"))
                cfg.vertical_shift_px = align["vertical_shift_px"].asInt();
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

    if (root.isMember("ar") && root["ar"].isObject()) {
        const Json::Value& ar = root["ar"];
        cfg.ar.enabled = ar.get("enabled", true).asBool();
        if (ar.isMember("left_socket") && ar["left_socket"].isString()) {
            cfg.ar.left_socket = ar["left_socket"].asString();
        }
        if (ar.isMember("right_socket") && ar["right_socket"].isString()) {
            cfg.ar.right_socket = ar["right_socket"].asString();
        }
        if (ar.isMember("color_key") && ar["color_key"].isArray() && ar["color_key"].size() == 3) {
            cfg.ar.use_color_key = true;
            cfg.ar.color_key_r = ar["color_key"][0].asInt();
            cfg.ar.color_key_g = ar["color_key"][1].asInt();
            cfg.ar.color_key_b = ar["color_key"][2].asInt();
        }
    }

    return cfg;
}

}  // namespace sv

#ifndef STEREO_VIEWER_COMMON_CONFIG_HPP
#define STEREO_VIEWER_COMMON_CONFIG_HPP

#include <json/json.h>

#include <string>
#include <vector>

namespace sv {

struct SourceConfig {
    std::string source;
};

struct ColorAdjustment {
    double brightness = 0.0;
    double contrast = 1.0;
    double saturation = 1.0;
    double hue = 0.0;
};

struct AppConfig {
    std::string name = "dvrk_stereo_viewer";
    std::string dvrk_console_namespace = "console";
    std::vector<std::string> ros_image_publishers;
    double overlay_alpha = 0.7;

    SourceConfig left;
    SourceConfig right;
    ColorAdjustment left_color;
    ColorAdjustment right_color;
    int original_width = 0;
    int original_height = 0;
    int crop_width = 0;
    int crop_height = 0;
    int horizontal_shift_px = 0;
    int vertical_shift_px = 0;
    bool preserve_size = true;
    std::vector<std::string> sinks;
    std::vector<std::string> sink_streams;
    bool has_unixfd_socket_path = true;
    std::string unixfd_socket_path = "";
};

class Config {
public:
    static bool load_from_file(const std::string& path, Json::Value& root);
    static bool check_type(const Json::Value& root, const std::string& expected_type, const std::string& path);
    static AppConfig parse_app_config(const Json::Value& root);
};

}  // namespace sv

#endif

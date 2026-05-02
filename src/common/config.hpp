#ifndef DVRK_DISPLAY_COMMON_CONFIG_HPP
#define DVRK_DISPLAY_COMMON_CONFIG_HPP

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

struct UnixfdSinkConfig {
    std::string stream;
    std::string name;
    std::string socket_path;
};

struct StereoExtraStream {
    std::string left;
    std::string right;
};

struct ExtraStreamsConfig {
    std::vector<std::string> monos;
    std::vector<StereoExtraStream> stereos;
    // Fraction of eye height used by the extra streams (0.0–1.0). Default 0.3.
    double scale = 0.3;
};

struct AppConfig {
    static constexpr int gap_px = 4;
    std::string name = "dvrk_display";
    std::string dvrk_console_namespace = "console";
    double overlay_alpha = 0.7;

    SourceConfig left;
    SourceConfig right;
    std::string stream; // For mono config
    std::string unixfd_socket_path;
    ColorAdjustment left_color;
    ColorAdjustment right_color;
    int original_width = 0;
    int original_height = 0;
    int crop_width = 0;
    int crop_height = 0;
    int horizontal_shift_px = 0;
    int vertical_shift_px = 0;
    int display_horizontal_offset_px = 0;
    bool preserve_size = true;
    std::vector<std::string> sinks;
    std::vector<std::string> sink_streams;
    std::vector<UnixfdSinkConfig> unixfd_sinks;
    ExtraStreamsConfig extra_streams;
};

class Config {
public:
    static bool load_from_file(const std::string& path, Json::Value& root);
    static bool check_type(const Json::Value& root, const std::string& expected_type, const std::string& path);
    static AppConfig parse_app_config(const Json::Value& root);
};

}  // namespace sv

#endif

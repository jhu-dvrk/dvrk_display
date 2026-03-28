#ifndef STEREO_VIEWER_COMMON_CONFIG_HPP
#define STEREO_VIEWER_COMMON_CONFIG_HPP

#include <json/json.h>

#include <string>
#include <vector>

namespace sv {

struct SourceConfig {
    std::string source;
};

struct AppConfig {
    std::string name = "dvrk_stereo_viewer";
    std::string dvrk_console_namespace = "console";
    std::vector<std::string> ros_image_publishers;
    double overlay_alpha = 0.7;

    SourceConfig left;
    SourceConfig right;
    int original_width = 0;
    int original_height = 0;
    int crop_width = 0;
    int crop_height = 0;
    int horizontal_shift_px = 0;
    int vertical_shift_px = 0;
    std::string sink_stream = "glimagesink sync=false force-aspect-ratio=false";
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

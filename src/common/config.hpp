#ifndef STEREO_VIEWER_COMMON_CONFIG_HPP
#define STEREO_VIEWER_COMMON_CONFIG_HPP

#include <json/json.h>

#include <string>

namespace sv {

struct SourceConfig {
    std::string source;
    int original_width = 0;
    int original_height = 0;
};

struct StereoConfig {
    std::string name;
    SourceConfig left;
    SourceConfig right;
    int original_width = 0;
    int original_height = 0;
    int crop_width = 0;
    int crop_height = 0;
    int horizontal_shift_px = 0;
    int vertical_shift_px = 0;
    std::string sink = "autovideosink sync=false";
    bool has_unixfd_socket_path = true;
    std::string unixfd_socket_path = "";
    double estimated_latency = 0.0;
};

struct AppConfig {
    std::string viewer_name = "dvrk_stereo_viewer";
    std::string console = "console";
    StereoConfig stereo;
};

class Config {
public:
    static bool load_from_file(const std::string& path, Json::Value& root);
    static bool check_type(const Json::Value& root, const std::string& expected_type, const std::string& path);
    static StereoConfig parse_stereo_config(const Json::Value& root);
    static AppConfig parse_app_config(const Json::Value& root);
};

}  // namespace sv

#endif

#ifndef DVRK_STEREO_VIEWER_OVERLAY_HPP
#define DVRK_STEREO_VIEWER_OVERLAY_HPP

#include <gst/gst.h>
#include <cairo/cairo.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <unordered_map>
#include <memory>
#include <mutex>
#include <string>

namespace sv {

enum class TeleopSide {
    Left,
    Right
};

struct TeleopIndicator {
    TeleopSide side = TeleopSide::Left;
    std::string arm_name;
    int psm_number = 0;
    bool is_camera_teleop = false;
    bool following_active = false;
};

struct ArmOverlayInfo {
    bool measured_cp_valid = true;
    std::string tool_type;
};

struct OverlayState {
    bool has_camera = false;
    bool has_clutch = false;
    bool camera_active = false;
    bool clutch_active = false;
    int frame_width = 0;
    int frame_height = 0;
    double overlay_alpha = 0.7;
    std::unordered_map<std::string, TeleopIndicator> teleop_indicators;
    std::unordered_map<std::string, ArmOverlayInfo> arm_info;
    std::mutex mutex;
};

bool parse_teleop_name(
    const std::string& teleop_name,
    std::string& mtm_name,
    TeleopSide& side,
    int& psm_number,
    std::string* arm_name = nullptr,
    bool* is_camera_teleop = nullptr
);

void on_teleop_selected(
    const std_msgs::msg::String::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
);

void on_teleop_unselected(
    const std_msgs::msg::String::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
);

void on_teleop_following(
    const std::string& teleop_name,
    const std_msgs::msg::Bool::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
);

void on_teleop_measured_cp(
    const std::string& psm_name,
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
);

void on_teleop_tool_type(
    const std::string& psm_name,
    const std_msgs::msg::String::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
);

void on_camera_joy(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
);

void on_clutch_joy(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
);

void on_overlay_caps_changed(GstElement*, GstCaps* caps, gpointer user_data);
void on_overlay_draw(GstElement*, cairo_t* cr, guint64, guint64, gpointer user_data);

}  // namespace sv

#endif

#include "overlay.hpp"

#include <gst/video/video.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <iomanip>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace sv {

std::string format_tool_type_label(const std::string& raw_tool_type);

namespace {

enum class OverlayView {
    Stereo,
    LeftEye,
    RightEye
};

OverlayView overlay_view_from_element(const GstElement* element) {
    if (element == nullptr) {
        return OverlayView::Stereo;
    }

    const char* element_name = GST_OBJECT_NAME(element);
    if (element_name == nullptr) {
        return OverlayView::Stereo;
    }

    const std::string name(element_name);
    if (name == "left_overlay") {
        return OverlayView::LeftEye;
    }
    if (name == "right_overlay") {
        return OverlayView::RightEye;
    }
    return OverlayView::Stereo;
}

int get_joy_state(const sensor_msgs::msg::Joy& msg) {
    bool has_two = false;
    for (const int value : msg.buttons) {
        if (value == 1) return 1;
        if (value == 2) has_two = true;
    }
    return has_two ? 2 : 0;
}

void draw_status_circle(cairo_t* cr, const int status, const double cx, const double cy, const double radius, const double alpha) {
    const double corner_radius = radius * 0.28;

    cairo_new_path(cr);
    cairo_move_to(cr, cx - radius, cy);
    cairo_arc(cr, cx, cy, radius, M_PI, 0.0);
    cairo_line_to(cr, cx + radius, cy + radius - corner_radius);
    cairo_arc(
        cr,
        cx + radius - corner_radius,
        cy + radius - corner_radius,
        corner_radius,
        0.0,
        0.5 * M_PI
    );
    cairo_line_to(cr, cx - radius + corner_radius, cy + radius);
    cairo_arc(
        cr,
        cx - radius + corner_radius,
        cy + radius - corner_radius,
        corner_radius,
        0.5 * M_PI,
        M_PI
    );
    cairo_close_path(cr);

    if (status != 0) {
        if (status == 2) {
            cairo_set_source_rgba(cr, 0.2, 0.8, 0.2, alpha);
        } else {
            cairo_set_source_rgba(cr, 0.75, 0.75, 0.75, alpha);
        }
        cairo_fill(cr);
    } else {
        cairo_set_source_rgba(cr, 0.75, 0.75, 0.75, alpha);
        cairo_set_line_width(cr, 2.5);
        cairo_stroke(cr);
    }
}

void draw_scale_gage(
    cairo_t* cr,
    const double scale,
    const bool on_right,
    const double cx,
    const double cy,
    const double radius,
    const double alpha
) {
    const double gage_height = radius * 2.0;
    const double gage_width = gage_height / 3.0;
    constexpr double k_gage_gap = 4.0;

    const double x = on_right
        ? (cx + radius + k_gage_gap)
        : (cx - radius - k_gage_gap - gage_width);
    const double y = cy - radius;

    // Background
    cairo_new_path(cr);
    cairo_rectangle(cr, x, y, gage_width, gage_height);
    cairo_set_source_rgba(cr, 0.15, 0.15, 0.15, alpha * 0.8);
    cairo_fill_preserve(cr);
    cairo_set_source_rgba(cr, 0.82, 0.82, 0.82, alpha);
    cairo_set_line_width(cr, 1.0);
    cairo_stroke(cr);

    // Fill
    const double clamped_scale = std::max(0.0, std::min(1.0, scale));
    const double fill_height = gage_height * clamped_scale;
    cairo_new_path(cr);
    cairo_rectangle(cr, x, y + gage_height - fill_height, gage_width, fill_height);
    cairo_set_source_rgba(cr, 0.75, 0.75, 0.75, alpha);
    cairo_fill(cr);
}

void draw_numbered_circle(
    cairo_t* cr,
    const bool active,
    const bool valid,
    const int number,
    const double cx,
    const double cy,
    const double radius,
    const double alpha
) {
    const double red_r = 0.9;
    const double red_g = 0.15;
    const double red_b = 0.15;

    cairo_new_path(cr);
    cairo_arc(cr, cx, cy, radius, 0.0, 2.0 * M_PI);
    if (active) {
        if (valid) {
            cairo_set_source_rgba(cr, 0.75, 0.75, 0.75, alpha);
        } else {
            cairo_set_source_rgba(cr, red_r, red_g, red_b, alpha);
        }
        cairo_fill_preserve(cr);
    }

    if (valid) {
        cairo_set_source_rgba(cr, 0.82, 0.82, 0.82, alpha);
    } else {
        cairo_set_source_rgba(cr, red_r, red_g, red_b, alpha);
    }
    cairo_set_line_width(cr, 2.5);
    cairo_stroke(cr);

    const std::string label = std::to_string(number);
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, radius * 1.1);

    cairo_text_extents_t extents;
    cairo_text_extents(cr, label.c_str(), &extents);
    cairo_move_to(
        cr,
        cx - (extents.width / 2.0 + extents.x_bearing),
        cy - (extents.height / 2.0 + extents.y_bearing)
    );
    cairo_set_source_rgba(cr, 0.98, 0.98, 0.98, alpha);
    cairo_show_text(cr, label.c_str());
}

void draw_tool_type_label(
    cairo_t* cr,
    const std::string& tool_type,
    const bool left_side,
    const double cx,
    const double cy,
    const double radius,
    const double alpha,
    const double extra_offset = 0.0
) {
    const std::string display_tool_type = format_tool_type_label(tool_type);
    if (display_tool_type.empty()) {
        return;
    }

    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size(cr, radius * 0.9);

    cairo_text_extents_t extents;
    cairo_text_extents(cr, display_tool_type.c_str(), &extents);

    constexpr double k_label_gap = 12.0;
    constexpr double k_label_lower_offset = 0.75;
    const double text_x = left_side
        ? (cx + radius + k_label_gap + extra_offset)
        : (cx - radius - k_label_gap - extents.width - extents.x_bearing - extra_offset);
    const double text_y = cy + radius * k_label_lower_offset - (extents.height / 2.0 + extents.y_bearing);

    cairo_move_to(cr, text_x, text_y);
    cairo_set_source_rgba(cr, 0.95, 0.95, 0.95, alpha);
    cairo_show_text(cr, display_tool_type.c_str());
}

void draw_scale_label(
    cairo_t* cr,
    const std::string& state,
    const bool left_side,
    const double cx,
    const double cy,
    const double radius,
    const double alpha,
    const double extra_offset = 0.0
) {
    std::string label = "";
    if (state == "DISABLED") {
        label = "disabled";
    } else if (state == "ALIGNING_MTM") {
        label = "Aligning...";
    } else if (state == "SETTING_ARMS_STATE") {
        label = "Checking arms";
    }

    if (label.empty()) {
        return;
    }

    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, radius * 0.75);

    cairo_text_extents_t extents;
    cairo_text_extents(cr, label.c_str(), &extents);

    constexpr double k_label_gap = 6.0;
    constexpr double k_label_upper_offset = -0.75;
    const double text_x = left_side
        ? (cx + radius + k_label_gap + extra_offset)
        : (cx - radius - k_label_gap - extents.width - extents.x_bearing - extra_offset);
    const double text_y = cy + radius * k_label_upper_offset - (extents.height / 2.0 + extents.y_bearing);

    cairo_move_to(cr, text_x, text_y);
    cairo_set_source_rgba(cr, 0.95, 0.95, 0.95, alpha);
    cairo_show_text(cr, label.c_str());
}

void draw_camera_icon(
    cairo_t* cr,
    const bool active,
    const bool valid,
    const double cx,
    const double cy,
    const double radius,
    const double alpha
) {
    const double outline_r = valid ? 0.82 : 0.9;
    const double outline_g = valid ? 0.82 : 0.15;
    const double outline_b = valid ? 0.82 : 0.15;

    const double body_height = radius * 1.6;
    const double body_width = body_height * (5.0 / 3.0);  // 5:3 horizontal rectangle
    const double x_left = cx - body_width * 0.5;
    const double y_top = cy - body_height * 0.5;
    const double corner_radius = body_height * 0.12;

    cairo_new_path(cr);
    cairo_move_to(cr, x_left + corner_radius, y_top);
    cairo_line_to(cr, x_left + body_width - corner_radius, y_top);
    cairo_arc(cr, x_left + body_width - corner_radius, y_top + corner_radius, corner_radius, -0.5 * M_PI, 0.0);
    cairo_line_to(cr, x_left + body_width, y_top + body_height - corner_radius);
    cairo_arc(cr, x_left + body_width - corner_radius, y_top + body_height - corner_radius, corner_radius, 0.0, 0.5 * M_PI);
    cairo_line_to(cr, x_left + corner_radius, y_top + body_height);
    cairo_arc(cr, x_left + corner_radius, y_top + body_height - corner_radius, corner_radius, 0.5 * M_PI, M_PI);
    cairo_line_to(cr, x_left, y_top + corner_radius);
    cairo_arc(cr, x_left + corner_radius, y_top + corner_radius, corner_radius, M_PI, 1.5 * M_PI);
    cairo_close_path(cr);

    if (active) {
        cairo_set_source_rgba(cr, 0.75, 0.75, 0.75, alpha);
        cairo_fill_preserve(cr);
    }

    cairo_set_source_rgba(cr, outline_r, outline_g, outline_b, alpha);
    cairo_set_line_width(cr, 2.5);
    cairo_stroke(cr);

    cairo_new_path(cr);
    cairo_arc(cr, cx, cy, body_height * 0.45, 0.0, 2.0 * M_PI);
    cairo_set_source_rgba(cr, outline_r, outline_g, outline_b, alpha);
    cairo_set_line_width(cr, 2.0);
    cairo_stroke(cr);
}

void draw_operator_present_icon(
    cairo_t* cr,
    const int status,
    const double cx,
    const double cy,
    const double radius,
    const double alpha
) {
    const double outline_r = 0.82;
    const double outline_g = 0.82;
    const double outline_b = 0.82;

    const double body_height = radius * 1.6;
    const double body_width = body_height * (5.0 / 3.0);
    const double x_left = cx - body_width * 0.5;
    const double y_top = cy - body_height * 0.5;
    const double corner_radius = body_height * 0.12;

    cairo_new_path(cr);
    cairo_move_to(cr, x_left + corner_radius, y_top);
    cairo_line_to(cr, x_left + body_width - corner_radius, y_top);
    cairo_arc(cr, x_left + body_width - corner_radius, y_top + corner_radius, corner_radius, -0.5 * M_PI, 0.0);
    cairo_line_to(cr, x_left + body_width, y_top + body_height - corner_radius);
    cairo_arc(cr, x_left + body_width - corner_radius, y_top + body_height - corner_radius, corner_radius, 0.0, 0.5 * M_PI);
    cairo_line_to(cr, x_left + corner_radius, y_top + body_height);
    cairo_arc(cr, x_left + corner_radius, y_top + body_height - corner_radius, corner_radius, 0.5 * M_PI, M_PI);
    cairo_line_to(cr, x_left, y_top + corner_radius);
    cairo_arc(cr, x_left + corner_radius, y_top + corner_radius, corner_radius, M_PI, 1.5 * M_PI);
    cairo_close_path(cr);

    if (status != 0) {
        if (status == 2) {
            cairo_set_source_rgba(cr, 0.2, 0.8, 0.2, alpha);
        } else {
            cairo_set_source_rgba(cr, 0.75, 0.75, 0.75, alpha);
        }
        cairo_fill_preserve(cr);
    }

    cairo_set_source_rgba(cr, outline_r, outline_g, outline_b, alpha);
    cairo_set_line_width(cr, 2.5);
    cairo_stroke(cr);

    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, radius * 0.9);
    cairo_text_extents_t extents;
    cairo_text_extents(cr, "OP", &extents);
    cairo_move_to(
        cr,
        cx - (extents.width / 2.0 + extents.x_bearing),
        cy - (extents.height / 2.0 + extents.y_bearing)
    );
    if (status != 0) {
        cairo_set_source_rgba(cr, 0.1, 0.1, 0.1, alpha);
    } else {
        cairo_set_source_rgba(cr, outline_r, outline_g, outline_b, alpha);
    }
    cairo_show_text(cr, "OP");
}

}  // namespace

bool parse_teleop_name(
    const std::string& teleop_name,
    std::string& mtm_name,
    TeleopSide& side,
    int& psm_number,
    std::string* arm_name,
    bool* is_camera_teleop
) {
    static const std::regex psm_teleop_regex(R"(^([A-Z0-9]+)_PSM([0-9]+)$)");
    static const std::regex ecm_teleop_regex(R"(^([A-Z0-9_]+)_ECM$)");

    if (is_camera_teleop != nullptr) {
        *is_camera_teleop = false;
    }

    std::smatch match;
    if (std::regex_match(teleop_name, match, psm_teleop_regex)) {
        mtm_name = match[1].str();
        if (mtm_name.rfind("MTML", 0) == 0) {
            side = TeleopSide::Left;
        } else if (mtm_name.rfind("MTMR", 0) == 0) {
            side = TeleopSide::Right;
        } else {
            return false;
        }

        try {
            psm_number = std::stoi(match[2].str());
        } catch (...) {
            return false;
        }
        if (psm_number <= 0) {
            return false;
        }

        if (arm_name != nullptr) {
            *arm_name = "PSM" + std::to_string(psm_number);
        }
        return true;
    }

    if (!std::regex_match(teleop_name, match, ecm_teleop_regex)) {
        return false;
    }

    mtm_name = match[1].str();
    if (mtm_name == "MTMR_MTML") {
        side = TeleopSide::Right;
    } else if (mtm_name == "MTML_MTMR") {
        side = TeleopSide::Left;
    } else {
        return false;
    }

    psm_number = 0;
    if (arm_name != nullptr) {
        *arm_name = "ECM";
    }
    if (is_camera_teleop != nullptr) {
        *is_camera_teleop = true;
    }
    return true;
}

bool parse_psm_name(const std::string& psm_name, int& psm_number) {
    static const std::regex psm_regex(R"(^PSM([0-9]+)$)");
    std::smatch match;
    if (!std::regex_match(psm_name, match, psm_regex)) {
        return false;
    }

    try {
        psm_number = std::stoi(match[1].str());
    } catch (...) {
        return false;
    }

    return psm_number > 0;
}

std::string format_tool_type_label(const std::string& raw_tool_type) {
    if (raw_tool_type.empty()) {
        return "";
    }

    static const std::regex prefix_regex(R"(^([^:]+):?.*$)");
    std::smatch match;
    if (!std::regex_match(raw_tool_type, match, prefix_regex)) {
        return "";
    }

    std::string formatted = match[1].str();
    std::replace(formatted.begin(), formatted.end(), '_', ' ');

    bool capitalize_next = true;
    for (auto& c : formatted) {
        const unsigned char uc = static_cast<unsigned char>(c);
        if (std::isspace(uc)) {
            capitalize_next = true;
            continue;
        }

        if (capitalize_next) {
            c = static_cast<char>(std::toupper(uc));
            capitalize_next = false;
        } else {
            c = static_cast<char>(std::tolower(uc));
        }
    }

    return formatted;
}

void on_teleop_selected(
    const std_msgs::msg::String::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    if (msg == nullptr) {
        return;
    }

    std::string mtm_name;
    TeleopSide side;
    int psm_number = 0;
    std::string arm_name;
    bool is_camera_teleop = false;
    if (!parse_teleop_name(msg->data, mtm_name, side, psm_number, &arm_name, &is_camera_teleop)) {
        return;
    }

    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    auto& indicator = overlay_state->teleop_indicators[msg->data];
    indicator.side = side;
    indicator.arm_name = arm_name;
    indicator.psm_number = psm_number;
    indicator.is_camera_teleop = is_camera_teleop;
}

void on_teleop_unselected(
    const std_msgs::msg::String::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    if (msg == nullptr) {
        return;
    }

    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    overlay_state->teleop_indicators.erase(msg->data);
}

void on_teleop_following(
    const std::string& teleop_name,
    const std_msgs::msg::Bool::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    if (msg == nullptr) {
        return;
    }

    std::string mtm_name;
    TeleopSide side;
    int psm_number = 0;
    std::string arm_name;
    bool is_camera_teleop = false;
    if (!parse_teleop_name(teleop_name, mtm_name, side, psm_number, &arm_name, &is_camera_teleop)) {
        return;
    }

    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    auto& indicator = overlay_state->teleop_indicators[teleop_name];
    indicator.side = side;
    indicator.arm_name = arm_name;
    indicator.psm_number = psm_number;
    indicator.is_camera_teleop = is_camera_teleop;
    indicator.following_active = msg->data;
}

void on_teleop_scale(
    const std::string& teleop_name,
    const std_msgs::msg::Float64::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    if (msg == nullptr) {
        return;
    }

    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    auto it = overlay_state->teleop_indicators.find(teleop_name);
    if (it != overlay_state->teleop_indicators.end()) {
        it->second.scale = msg->data;
    }
}

void on_teleop_current_state(
    const std::string& teleop_name,
    const std_msgs::msg::String::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    if (msg == nullptr) {
        return;
    }

    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    auto it = overlay_state->teleop_indicators.find(teleop_name);
    if (it != overlay_state->teleop_indicators.end()) {
        it->second.current_state = msg->data;
    }
}

void on_teleop_measured_cp(
    const std::string& psm_name,
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    if (msg == nullptr) {
        return;
    }

    if (psm_name.empty()) {
        return;
    }

    const bool valid = (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    overlay_state->arm_info[psm_name].measured_cp_valid = valid;
}

void on_teleop_tool_type(
    const std::string& psm_name,
    const std_msgs::msg::String::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    if (msg == nullptr) {
        return;
    }

    int psm_number = 0;
    if (!parse_psm_name(psm_name, psm_number)) {
        return;
    }

    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    overlay_state->arm_info[psm_name].tool_type = msg->data;
}

void update_button_state(const sensor_msgs::msg::Joy::SharedPtr msg, ButtonState& btn) {
    if (msg == nullptr) return;
    btn.present = true;
    int state = get_joy_state(*msg);
    if (state == 1) {
        btn.active = true;
    } else if (state == 2) {
        btn.expiration = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    } else {
        btn.active = false;
    }
}

void on_camera_joy(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    update_button_state(msg, overlay_state->camera);
}

void on_clutch_joy(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    update_button_state(msg, overlay_state->clutch);
}

void on_operator_present(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    update_button_state(msg, overlay_state->operator_present);
}

void on_overlay_caps_changed(GstElement*, GstCaps* caps, gpointer user_data) {
    if (caps == nullptr || user_data == nullptr) {
        return;
    }

    GstVideoInfo video_info;
    if (!gst_video_info_from_caps(&video_info, caps)) {
        return;
    }

    auto* overlay_state = static_cast<OverlayState*>(user_data);
    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    overlay_state->frame_width = static_cast<int>(video_info.width);
    overlay_state->frame_height = static_cast<int>(video_info.height);
}

void on_overlay_draw(GstElement* overlay, cairo_t* cr, guint64, guint64, gpointer user_data) {
    if (cr == nullptr || user_data == nullptr) {
        return;
    }

    auto* overlay_state = static_cast<OverlayState*>(user_data);
    int camera_status = 0;
    int clutch_status = 0;
    bool has_operator_present = false;
    int operator_present_status = 0;
    int frame_width = 0;
    int frame_height = 0;
    double overlay_alpha = 0.7;
    int display_horizontal_offset_px = 0;
    std::unordered_map<std::string, ArmOverlayInfo> arm_info;
    std::vector<TeleopIndicator> left_teleops;
    std::vector<TeleopIndicator> right_teleops;
    bool has_camera_teleop = false;
    TeleopIndicator camera_teleop;

    {
        std::scoped_lock<std::mutex> lock(overlay_state->mutex);
        if (!overlay_state->overlay_enabled) {
            return;
        }
        camera_status = overlay_state->camera.present ? overlay_state->camera.get_status() : 0;
        clutch_status = overlay_state->clutch.present ? overlay_state->clutch.get_status() : 0;
        has_operator_present = overlay_state->operator_present.present;
        operator_present_status = overlay_state->operator_present.present ? overlay_state->operator_present.get_status() : 0;
        frame_width = overlay_state->frame_width;
        frame_height = overlay_state->frame_height;
        overlay_alpha = overlay_state->overlay_alpha;
        display_horizontal_offset_px = overlay_state->display_horizontal_offset_px;
        arm_info = overlay_state->arm_info;

        for (const auto& [_, indicator] : overlay_state->teleop_indicators) {
            if (indicator.is_camera_teleop) {
                if (!has_camera_teleop) {
                    camera_teleop = indicator;
                    has_camera_teleop = true;
                }
                continue;
            }

            if (indicator.psm_number <= 0 || indicator.arm_name.empty()) {
                continue;
            }
            if (indicator.side == TeleopSide::Left) {
                left_teleops.push_back(indicator);
            } else {
                right_teleops.push_back(indicator);
            }
        }
    }

    if (frame_width <= 0 || frame_height <= 0) {
        return;
    }

    const OverlayView overlay_view = overlay_view_from_element(overlay);
    const bool is_stereo_layout = overlay_view == OverlayView::Stereo;
    const double eye_width = is_stereo_layout ? static_cast<double>(frame_width) / 2.0 : static_cast<double>(frame_width);
    const double image_scale = std::min(eye_width, static_cast<double>(frame_height));

    const double horizontal_ui_scale = (eye_width > 0) ? (eye_width - 2.0 * std::abs(static_cast<double>(display_horizontal_offset_px))) / eye_width : 1.0;

    // Ratios are based on previous pixel constants halved and normalized by a reference eye height (~425 px).
    constexpr double k_status_radius_ratio = 6.4 / 425.0;
    constexpr double k_status_spacing_ratio = 15.0 / 425.0;
    constexpr double k_overlay_margin_ratio = 12.0 / 425.0;
    constexpr double k_psm_radius_ratio = 7.0 / 425.0;
    constexpr double k_psm_x_margin_ratio = 30.0 / 425.0;
    constexpr double k_psm_y_step_ratio = 20.0 / 425.0;

    const double radius = image_scale * k_status_radius_ratio;
    const double spacing = image_scale * k_status_spacing_ratio;
    const double bottom_margin = image_scale * k_overlay_margin_ratio;
    const double cy = static_cast<double>(frame_height) - bottom_margin;

    const double psm_radius = image_scale * k_psm_radius_ratio;
    const double psm_x_margin = image_scale * k_psm_x_margin_ratio;
    const double psm_y_step = image_scale * k_psm_y_step_ratio;
    const double psm_base_y = cy;
    const double top_margin = image_scale * k_overlay_margin_ratio;

    // Draw background strips
    cairo_set_source_rgba(cr, 0.1, 0.1, 0.1, 0.5);
    // Bottom strip (full width)
    cairo_rectangle(cr, 0, frame_height - 2.0 * bottom_margin, frame_width, 2.0 * bottom_margin);
    cairo_fill(cr);

    if (is_stereo_layout) {
        const double left_baseline_cx = (eye_width / 2.0) - static_cast<double>(display_horizontal_offset_px) / 2.0;
        const double left_cx = left_baseline_cx;
        draw_status_circle(cr, clutch_status, left_cx - spacing * horizontal_ui_scale, cy, radius, overlay_alpha);
        draw_status_circle(cr, camera_status, left_cx + spacing * horizontal_ui_scale, cy, radius, overlay_alpha);

        const double right_baseline_cx = eye_width + (eye_width / 2.0) + static_cast<double>(display_horizontal_offset_px) / 2.0;
        const double right_cx = right_baseline_cx;
        draw_status_circle(cr, clutch_status, right_cx - spacing * horizontal_ui_scale, cy, radius, overlay_alpha);
        draw_status_circle(cr, camera_status, right_cx + spacing * horizontal_ui_scale, cy, radius, overlay_alpha);
    } else {
        const double center_cx = static_cast<double>(frame_width) / 2.0;
        draw_status_circle(cr, clutch_status, center_cx - spacing, cy, radius, overlay_alpha);
        draw_status_circle(cr, camera_status, center_cx + spacing, cy, radius, overlay_alpha);
    }

    std::sort(left_teleops.begin(), left_teleops.end(), [](const TeleopIndicator& a, const TeleopIndicator& b) {
        return a.psm_number < b.psm_number;
    });
    std::sort(right_teleops.begin(), right_teleops.end(), [](const TeleopIndicator& a, const TeleopIndicator& b) {
        return a.psm_number < b.psm_number;
    });

    const bool show_op = has_operator_present;
    const bool show_cam = has_camera_teleop;

    if (show_op || show_cam) {
        bool camera_valid = true;
        if (show_cam) {
            camera_valid = camera_teleop.arm_name.empty()
                ? true
                : (arm_info.count(camera_teleop.arm_name) > 0
                       ? arm_info[camera_teleop.arm_name].measured_cp_valid
                       : true);
        }

        auto draw_top_icons = [&](const double cx) {
            const double top_icon_margin = top_margin * 0.4;
            const double cy_top = top_icon_margin + psm_radius * 0.8;
            const double top_spacing = psm_radius * 2.0; // wider spacing for rectangular icons
            const double body_width = psm_radius * 1.6 * (5.0 / 3.0);
            
            double bg_x_left = cx;
            double bg_width = 0;
            
            if (show_op && show_cam) {
                bg_x_left = cx - top_spacing - body_width * 0.5;
                bg_width = 2.0 * top_spacing + body_width;
            } else if (show_op || show_cam) {
                bg_x_left = cx - body_width * 0.5;
                bg_width = body_width;
            }
            
            const double bg_padding_x = psm_radius * 0.8;
            const double bg_padding_bottom = psm_radius * 0.6;
            
            cairo_set_source_rgba(cr, 0.1, 0.1, 0.1, 0.2);
            cairo_rectangle(cr, 
                bg_x_left - bg_padding_x, 
                0, 
                bg_width + 2.0 * bg_padding_x, 
                cy_top + psm_radius * 0.8 + bg_padding_bottom
            );
            cairo_fill(cr);

            if (show_op && show_cam) {
                draw_operator_present_icon(cr, operator_present_status, cx - top_spacing, cy_top, psm_radius, overlay_alpha);
                draw_camera_icon(cr, camera_teleop.following_active, camera_valid, cx + top_spacing, cy_top, psm_radius, overlay_alpha);
            } else if (show_op) {
                draw_operator_present_icon(cr, operator_present_status, cx, cy_top, psm_radius, overlay_alpha);
            } else if (show_cam) {
                draw_camera_icon(cr, camera_teleop.following_active, camera_valid, cx, cy_top, psm_radius, overlay_alpha);
            }
        };

        if (is_stereo_layout) {
            for (int eye_index = 0; eye_index < 2; ++eye_index) {
                const double offset = (eye_index == 0) ? -static_cast<double>(display_horizontal_offset_px) / 2.0 : static_cast<double>(display_horizontal_offset_px) / 2.0;
                const double eye_center_x = (eye_width * (static_cast<double>(eye_index) + 0.5)) + offset;
                draw_top_icons(eye_center_x);
            }
        } else {
            draw_top_icons(static_cast<double>(frame_width) / 2.0);
        }
    }

    const auto draw_teleop_column = [&](const std::vector<TeleopIndicator>& teleops, const double x, const bool label_on_right) {
        for (size_t index = 0; index < teleops.size(); ++index) {
            const double psm_y = psm_base_y - static_cast<double>(index) * psm_y_step;
            draw_numbered_circle(
                cr,
                teleops[index].following_active,
                arm_info.count(teleops[index].arm_name) > 0
                    ? arm_info[teleops[index].arm_name].measured_cp_valid
                    : true,
                teleops[index].psm_number,
                x,
                psm_y,
                psm_radius,
                overlay_alpha
            );

            draw_scale_gage(
                cr,
                teleops[index].scale,
                label_on_right,
                x,
                psm_y,
                psm_radius,
                overlay_alpha
            );

            const double gage_width = (psm_radius * 2.0) / 3.0;
            const double extra_offset = 4.0 + gage_width;

            const auto info_it = arm_info.find(teleops[index].arm_name);
            if (info_it != arm_info.end()) {
                draw_tool_type_label(
                    cr,
                    info_it->second.tool_type,
                    label_on_right,
                    x,
                    psm_y,
                    psm_radius,
                    overlay_alpha,
                    extra_offset
                );
            }

            draw_scale_label(
                cr,
                teleops[index].current_state,
                label_on_right,
                x,
                psm_y,
                psm_radius,
                overlay_alpha,
                extra_offset
            );
        }
    };

    if (is_stereo_layout) {
        for (int eye_index = 0; eye_index < 2; ++eye_index) {
            const double eye_x_offset = eye_width * static_cast<double>(eye_index);
            const double shift = (eye_index == 0) ? -static_cast<double>(display_horizontal_offset_px) / 2.0 : static_cast<double>(display_horizontal_offset_px) / 2.0;

            const double psm_left_x = eye_x_offset + (eye_width / 2.0) + shift - (eye_width / 2.0) * horizontal_ui_scale + psm_x_margin * horizontal_ui_scale;
            const double psm_right_x = eye_x_offset + (eye_width / 2.0) + shift + (eye_width / 2.0) * horizontal_ui_scale - psm_x_margin * horizontal_ui_scale;
            draw_teleop_column(left_teleops, psm_left_x, true);
            draw_teleop_column(right_teleops, psm_right_x, false);
        }
    } else {
        // Single-eye layout: both PSM columns shown in every eye, only corner dot differs.
        constexpr double k_eye_dot_radius_ratio = 2.0 / 425.0;
        const double dot_r = image_scale * k_eye_dot_radius_ratio;
        const double dot_x = overlay_view == OverlayView::LeftEye
            ? dot_r
            : static_cast<double>(frame_width) - dot_r;
        cairo_new_path(cr);
        cairo_arc(cr, dot_x, dot_r, dot_r, 0.0, 2.0 * M_PI);
        cairo_set_source_rgba(cr, 0.82, 0.82, 0.82, overlay_alpha);
        cairo_fill(cr);
        draw_teleop_column(left_teleops, psm_x_margin, true);
        draw_teleop_column(right_teleops, static_cast<double>(frame_width) - psm_x_margin, false);
    }
}

}  // namespace sv

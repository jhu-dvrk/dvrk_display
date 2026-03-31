#include "overlay.hpp"

#include <gst/video/video.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <regex>
#include <string>
#include <vector>

namespace sv {

std::string format_tool_type_label(const std::string& raw_tool_type);

namespace {

bool joy_active(const sensor_msgs::msg::Joy& msg) {
    const bool any_button_pressed = std::any_of(
        msg.buttons.begin(), msg.buttons.end(),
        [](const int value) {
            return value != 0;
        }
    );
    if (any_button_pressed) {
        return true;
    }

    return std::any_of(
        msg.axes.begin(), msg.axes.end(),
        [](const float value) {
            return std::abs(value) > 0.5F;
        }
    );
}

void draw_status_circle(cairo_t* cr, const bool active, const double cx, const double cy, const double radius, const double alpha) {
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

    if (active) {
        cairo_set_source_rgba(cr, 0.75, 0.75, 0.75, alpha);
        cairo_fill(cr);
    } else {
        cairo_set_source_rgba(cr, 0.75, 0.75, 0.75, alpha);
        cairo_set_line_width(cr, 2.5);
        cairo_stroke(cr);
    }
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
    const double alpha
) {
    const std::string display_tool_type = format_tool_type_label(tool_type);
    if (display_tool_type.empty()) {
        return;
    }

    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size(cr, radius * 0.9);

    cairo_text_extents_t extents;
    cairo_text_extents(cr, display_tool_type.c_str(), &extents);

    constexpr double k_label_gap = 6.0;
    constexpr double k_label_lower_offset = 0.75;
    const double text_x = left_side
        ? (cx + radius + k_label_gap)
        : (cx - radius - k_label_gap - extents.width - extents.x_bearing);
    const double text_y = cy + radius * k_label_lower_offset - (extents.height / 2.0 + extents.y_bearing);

    cairo_move_to(cr, text_x, text_y);
    cairo_set_source_rgba(cr, 0.95, 0.95, 0.95, alpha);
    cairo_show_text(cr, display_tool_type.c_str());
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

void on_camera_joy(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    overlay_state->has_camera = true;
    overlay_state->camera_active = joy_active(*msg);
}

void on_clutch_joy(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    overlay_state->has_clutch = true;
    overlay_state->clutch_active = joy_active(*msg);
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

void on_overlay_draw(GstElement*, cairo_t* cr, guint64, guint64, gpointer user_data) {
    if (cr == nullptr || user_data == nullptr) {
        return;
    }

    auto* overlay_state = static_cast<OverlayState*>(user_data);
    bool camera_active = false;
    bool clutch_active = false;
    bool is_stereo = true;
    int frame_width = 0;
    int frame_height = 0;
    double overlay_alpha = 0.7;
    std::unordered_map<std::string, ArmOverlayInfo> arm_info;
    std::vector<TeleopIndicator> left_teleops;
    std::vector<TeleopIndicator> right_teleops;
    bool has_camera_teleop = false;
    TeleopIndicator camera_teleop;

    {
        std::scoped_lock<std::mutex> lock(overlay_state->mutex);
        camera_active = overlay_state->has_camera && overlay_state->camera_active;
        clutch_active = overlay_state->has_clutch && overlay_state->clutch_active;
        is_stereo = overlay_state->is_stereo;
        frame_width = overlay_state->frame_width;
        frame_height = overlay_state->frame_height;
        overlay_alpha = overlay_state->overlay_alpha;
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

    const double eye_width = is_stereo ? static_cast<double>(frame_width) / 2.0 : static_cast<double>(frame_width);
    const double image_scale = std::min(eye_width, static_cast<double>(frame_height));

    // Ratios are based on previous pixel constants halved and normalized by a reference eye height (~425 px).
    constexpr double k_status_radius_ratio = 10.0 / 425.0;
    constexpr double k_status_spacing_ratio = 15.0 / 425.0;
    constexpr double k_overlay_margin_ratio = 16.0 / 425.0;
    constexpr double k_psm_radius_ratio = 11.0 / 425.0;
    constexpr double k_psm_x_margin_ratio = 27.5 / 425.0;
    constexpr double k_psm_y_step_ratio = 26.0 / 425.0;

    const double radius = image_scale * k_status_radius_ratio;
    const double spacing = image_scale * k_status_spacing_ratio;
    const double bottom_margin = image_scale * k_overlay_margin_ratio;
    const double cy = static_cast<double>(frame_height) - bottom_margin;

    const double left_cx = eye_width / 2.0;
    draw_status_circle(cr, clutch_active, left_cx - spacing, cy, radius, overlay_alpha);
    draw_status_circle(cr, camera_active, left_cx + spacing, cy, radius, overlay_alpha);

    if (is_stereo) {
        const double right_cx = 3.0 * eye_width / 2.0;
        draw_status_circle(cr, clutch_active, right_cx - spacing, cy, radius, overlay_alpha);
        draw_status_circle(cr, camera_active, right_cx + spacing, cy, radius, overlay_alpha);
    }

    std::sort(left_teleops.begin(), left_teleops.end(), [](const TeleopIndicator& a, const TeleopIndicator& b) {
        return a.psm_number < b.psm_number;
    });
    std::sort(right_teleops.begin(), right_teleops.end(), [](const TeleopIndicator& a, const TeleopIndicator& b) {
        return a.psm_number < b.psm_number;
    });

    const double psm_radius = image_scale * k_psm_radius_ratio;
    const double psm_x_margin = image_scale * k_psm_x_margin_ratio;
    const double psm_y_step = image_scale * k_psm_y_step_ratio;
    const double psm_base_y = cy;
    const double top_margin = image_scale * k_overlay_margin_ratio;

    if (has_camera_teleop) {
        const bool camera_valid = camera_teleop.arm_name.empty()
            ? true
            : (arm_info.count(camera_teleop.arm_name) > 0
                   ? arm_info[camera_teleop.arm_name].measured_cp_valid
                   : true);
        const int num_eyes = is_stereo ? 2 : 1;
        for (int eye_index = 0; eye_index < num_eyes; ++eye_index) {
            const double eye_center_x = eye_width * (static_cast<double>(eye_index) + 0.5);
            draw_camera_icon(
                cr,
                camera_teleop.following_active,
                camera_valid,
                eye_center_x,
                top_margin + psm_radius,
                psm_radius,
                overlay_alpha
            );
        }
    }

    const int num_eyes = is_stereo ? 2 : 1;
    for (int eye_index = 0; eye_index < num_eyes; ++eye_index) {
        const double eye_x_offset = eye_width * static_cast<double>(eye_index);
        const double psm_left_x = eye_x_offset + psm_x_margin;
        const double psm_right_x = eye_x_offset + eye_width - psm_x_margin;

        for (size_t index = 0; index < left_teleops.size(); ++index) {
            const double psm_y = psm_base_y - static_cast<double>(index) * psm_y_step;
            draw_numbered_circle(
                cr,
                left_teleops[index].following_active,
                arm_info.count(left_teleops[index].arm_name) > 0
                    ? arm_info[left_teleops[index].arm_name].measured_cp_valid
                    : true,
                left_teleops[index].psm_number,
                psm_left_x,
                psm_y,
                psm_radius,
                overlay_alpha
            );

            const auto info_it = arm_info.find(left_teleops[index].arm_name);
            if (info_it != arm_info.end()) {
                draw_tool_type_label(
                    cr,
                    info_it->second.tool_type,
                    true,
                    psm_left_x,
                    psm_y,
                    psm_radius,
                    overlay_alpha
                );
            }
        }

        for (size_t index = 0; index < right_teleops.size(); ++index) {
            const double psm_y = psm_base_y - static_cast<double>(index) * psm_y_step;
            draw_numbered_circle(
                cr,
                right_teleops[index].following_active,
                arm_info.count(right_teleops[index].arm_name) > 0
                    ? arm_info[right_teleops[index].arm_name].measured_cp_valid
                    : true,
                right_teleops[index].psm_number,
                psm_right_x,
                psm_y,
                psm_radius,
                overlay_alpha
            );

            const auto info_it = arm_info.find(right_teleops[index].arm_name);
            if (info_it != arm_info.end()) {
                draw_tool_type_label(
                    cr,
                    info_it->second.tool_type,
                    false,
                    psm_right_x,
                    psm_y,
                    psm_radius,
                    overlay_alpha
                );
            }
        }
    }
}

}  // namespace sv

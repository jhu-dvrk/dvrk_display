#include "overlay.hpp"

#include <gst/video/video.h>

#include <algorithm>
#include <cmath>
#include <regex>
#include <string>
#include <vector>

namespace sv {

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
    cairo_new_path(cr);
    cairo_arc(cr, cx, cy, radius, 0.0, 2.0 * M_PI);
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
    const int number,
    const double cx,
    const double cy,
    const double radius,
    const double alpha
) {
    cairo_new_path(cr);
    cairo_arc(cr, cx, cy, radius, 0.0, 2.0 * M_PI);
    if (active) {
        cairo_set_source_rgba(cr, 0.75, 0.75, 0.75, alpha);
        cairo_fill_preserve(cr);
    }

    cairo_set_source_rgba(cr, 0.82, 0.82, 0.82, alpha);
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

}  // namespace

bool parse_teleop_name(const std::string& teleop_name, std::string& mtm_name, TeleopSide& side, int& psm_number) {
    static const std::regex teleop_regex(R"(^([A-Z0-9]+)_PSM([0-9]+)$)");
    std::smatch match;
    if (!std::regex_match(teleop_name, match, teleop_regex)) {
        return false;
    }

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
    return psm_number > 0;
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
    if (!parse_teleop_name(msg->data, mtm_name, side, psm_number)) {
        return;
    }

    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    auto& indicator = overlay_state->teleop_indicators[msg->data];
    indicator.side = side;
    indicator.psm_number = psm_number;
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
    if (!parse_teleop_name(teleop_name, mtm_name, side, psm_number)) {
        return;
    }

    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    auto& indicator = overlay_state->teleop_indicators[teleop_name];
    indicator.side = side;
    indicator.psm_number = psm_number;
    indicator.following_active = msg->data;
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
    int frame_width = 0;
    int frame_height = 0;
    double overlay_alpha = 0.7;
    std::vector<TeleopIndicator> left_teleops;
    std::vector<TeleopIndicator> right_teleops;

    {
        std::scoped_lock<std::mutex> lock(overlay_state->mutex);
        camera_active = overlay_state->has_camera && overlay_state->camera_active;
        clutch_active = overlay_state->has_clutch && overlay_state->clutch_active;
        frame_width = overlay_state->frame_width;
        frame_height = overlay_state->frame_height;
        overlay_alpha = overlay_state->overlay_alpha;

        for (const auto& [_, indicator] : overlay_state->teleop_indicators) {
            if (indicator.psm_number <= 0) {
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

    const double eye_width = static_cast<double>(frame_width) / 2.0;
    const double image_scale = std::min(eye_width, static_cast<double>(frame_height));

    // Ratios are based on previous pixel constants halved and normalized by a reference eye height (~425 px).
    constexpr double k_status_radius_ratio = 10.0 / 425.0;
    constexpr double k_status_spacing_ratio = 15.0 / 425.0;
    constexpr double k_bottom_margin_ratio = 25.0 / 425.0;
    constexpr double k_psm_radius_ratio = 11.0 / 425.0;
    constexpr double k_psm_x_margin_ratio = 27.5 / 425.0;
    constexpr double k_psm_y_step_ratio = 26.0 / 425.0;

    const double radius = image_scale * k_status_radius_ratio;
    const double spacing = image_scale * k_status_spacing_ratio;
    const double bottom_margin = image_scale * k_bottom_margin_ratio;
    const double cy = static_cast<double>(frame_height) - bottom_margin;

    const double left_cx = static_cast<double>(frame_width) / 4.0;
    draw_status_circle(cr, clutch_active, left_cx - spacing, cy, radius, overlay_alpha);
    draw_status_circle(cr, camera_active, left_cx + spacing, cy, radius, overlay_alpha);

    const double right_cx = 3.0 * static_cast<double>(frame_width) / 4.0;
    draw_status_circle(cr, clutch_active, right_cx - spacing, cy, radius, overlay_alpha);
    draw_status_circle(cr, camera_active, right_cx + spacing, cy, radius, overlay_alpha);

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

    for (int eye_index = 0; eye_index < 2; ++eye_index) {
        const double eye_x_offset = eye_width * static_cast<double>(eye_index);
        const double psm_left_x = eye_x_offset + psm_x_margin;
        const double psm_right_x = eye_x_offset + eye_width - psm_x_margin;

        for (size_t index = 0; index < left_teleops.size(); ++index) {
            const double psm_y = psm_base_y - static_cast<double>(index) * psm_y_step;
            draw_numbered_circle(
                cr,
                left_teleops[index].following_active,
                left_teleops[index].psm_number,
                psm_left_x,
                psm_y,
                psm_radius,
                overlay_alpha
            );
        }

        for (size_t index = 0; index < right_teleops.size(); ++index) {
            const double psm_y = psm_base_y - static_cast<double>(index) * psm_y_step;
            draw_numbered_circle(
                cr,
                right_teleops[index].following_active,
                right_teleops[index].psm_number,
                psm_right_x,
                psm_y,
                psm_radius,
                overlay_alpha
            );
        }
    }
}

}  // namespace sv

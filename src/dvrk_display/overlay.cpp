#include "overlay.hpp"
#include "overlay_components.hpp"
#include "overlay_theme.hpp"
#include "overlay_utils.hpp"

#include <gst/video/video.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace sv {

namespace {

enum class OverlayView { Stereo, LeftEye, RightEye };

OverlayView overlay_view_from_element(const GstElement *element) {
  if (element == nullptr) {
    return OverlayView::Stereo;
  }

  const char *element_name = GST_OBJECT_NAME(element);
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

int get_joy_state(const sensor_msgs::msg::Joy &msg) {
  bool has_two = false;
  for (const int value : msg.buttons) {
    if (value == 1)
      return 1;
    if (value == 2)
      has_two = true;
  }
  return has_two ? 2 : 0;
}

} // namespace

bool parse_teleop_name(const std::string &teleop_name, std::string &mtm_name,
                       TeleopSide &side, int &psm_number, std::string *arm_name,
                       bool *is_camera_teleop) {
  if (is_camera_teleop != nullptr) {
    *is_camera_teleop = false;
  }

  const std::string ecm_suffix = "_ECM";
  if (teleop_name.size() > ecm_suffix.size() &&
      teleop_name.compare(teleop_name.size() - ecm_suffix.size(),
                          ecm_suffix.size(), ecm_suffix) == 0) {
    mtm_name = teleop_name.substr(0, teleop_name.size() - ecm_suffix.size());
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

  auto psm_pos = teleop_name.find("_PSM");
  if (psm_pos != std::string::npos && psm_pos > 0) {
    mtm_name = teleop_name.substr(0, psm_pos);
    if (mtm_name.rfind("MTML", 0) == 0) {
      side = TeleopSide::Left;
    } else if (mtm_name.rfind("MTMR", 0) == 0) {
      side = TeleopSide::Right;
    } else {
      return false;
    }

    std::string psm_num_str = teleop_name.substr(psm_pos + 4);
    if (psm_num_str.empty()) {
      return false;
    }
    for (char c : psm_num_str) {
      if (!std::isdigit(static_cast<unsigned char>(c))) {
        return false;
      }
    }
    try {
      psm_number = std::stoi(psm_num_str);
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

  return false;
}

bool parse_psm_name(const std::string &psm_name, int &psm_number) {
  if (psm_name.rfind("PSM", 0) != 0 || psm_name.size() <= 3) {
    return false;
  }

  std::string num_str = psm_name.substr(3);
  for (char c : num_str) {
    if (!std::isdigit(static_cast<unsigned char>(c))) {
      return false;
    }
  }

  try {
    psm_number = std::stoi(num_str);
  } catch (...) {
    return false;
  }

  return psm_number > 0;
}

void on_teleop_selected(const std_msgs::msg::String::SharedPtr msg,
                        const std::shared_ptr<OverlayState> &overlay_state) {
  if (msg == nullptr) {
    return;
  }

  std::string mtm_name;
  TeleopSide side;
  int psm_number = 0;
  std::string arm_name;
  bool is_camera_teleop = false;
  if (!parse_teleop_name(msg->data, mtm_name, side, psm_number, &arm_name,
                         &is_camera_teleop)) {
    return;
  }

  std::scoped_lock<std::mutex> lock(overlay_state->mutex);
  auto &indicator = overlay_state->teleop_indicators[msg->data];
  indicator.side = side;
  indicator.arm_name = arm_name;
  indicator.psm_number = psm_number;
  indicator.is_camera_teleop = is_camera_teleop;
}

void on_teleop_unselected(const std_msgs::msg::String::SharedPtr msg,
                          const std::shared_ptr<OverlayState> &overlay_state) {
  if (msg == nullptr) {
    return;
  }

  std::scoped_lock<std::mutex> lock(overlay_state->mutex);
  overlay_state->teleop_indicators.erase(msg->data);
}

void on_teleop_following(const std::string &teleop_name,
                         const std_msgs::msg::Bool::SharedPtr msg,
                         const std::shared_ptr<OverlayState> &overlay_state) {
  if (msg == nullptr) {
    return;
  }

  std::string mtm_name;
  TeleopSide side;
  int psm_number = 0;
  std::string arm_name;
  bool is_camera_teleop = false;
  if (!parse_teleop_name(teleop_name, mtm_name, side, psm_number, &arm_name,
                         &is_camera_teleop)) {
    return;
  }

  std::scoped_lock<std::mutex> lock(overlay_state->mutex);
  auto &indicator = overlay_state->teleop_indicators[teleop_name];
  indicator.side = side;
  indicator.arm_name = arm_name;
  indicator.psm_number = psm_number;
  indicator.is_camera_teleop = is_camera_teleop;
  indicator.following_active = msg->data;
}

void on_teleop_scale(const std::string &teleop_name,
                     const std_msgs::msg::Float64::SharedPtr msg,
                     const std::shared_ptr<OverlayState> &overlay_state) {
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
    const std::string &teleop_name, const std_msgs::msg::String::SharedPtr msg,
    const std::shared_ptr<OverlayState> &overlay_state) {
  if (msg == nullptr) {
    return;
  }

  std::scoped_lock<std::mutex> lock(overlay_state->mutex);
  auto it = overlay_state->teleop_indicators.find(teleop_name);
  if (it != overlay_state->teleop_indicators.end()) {
    it->second.current_state = msg->data;
  }
}

void on_teleop_measured_cp(const std::string &psm_name,
                           const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                           const std::shared_ptr<OverlayState> &overlay_state) {
  if (msg == nullptr) {
    return;
  }

  if (psm_name.empty()) {
    return;
  }

  const bool valid =
      (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
  std::scoped_lock<std::mutex> lock(overlay_state->mutex);
  overlay_state->arm_info[psm_name].measured_cp_valid = valid;
}

void on_teleop_tool_type(const std::string &psm_name,
                         const std_msgs::msg::String::SharedPtr msg,
                         const std::shared_ptr<OverlayState> &overlay_state) {
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

void update_button_state(const sensor_msgs::msg::Joy::SharedPtr msg,
                         ButtonState &btn) {
  if (msg == nullptr)
    return;
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

void on_camera_joy(const sensor_msgs::msg::Joy::SharedPtr msg,
                   const std::shared_ptr<OverlayState> &overlay_state) {
  std::scoped_lock<std::mutex> lock(overlay_state->mutex);
  update_button_state(msg, overlay_state->camera);
}

void on_clutch_joy(const sensor_msgs::msg::Joy::SharedPtr msg,
                   const std::shared_ptr<OverlayState> &overlay_state) {
  std::scoped_lock<std::mutex> lock(overlay_state->mutex);
  update_button_state(msg, overlay_state->clutch);
}

void on_operator_present(const sensor_msgs::msg::Joy::SharedPtr msg,
                         const std::shared_ptr<OverlayState> &overlay_state) {
  std::scoped_lock<std::mutex> lock(overlay_state->mutex);
  update_button_state(msg, overlay_state->operator_present);
}

void on_ecm_measured_js(const sensor_msgs::msg::JointState::SharedPtr msg,
                        const std::shared_ptr<OverlayState> &overlay_state) {
  if (msg == nullptr) {
    return;
  }
  if (msg->position.size() < 4) {
    return;
  }
  std::scoped_lock<std::mutex> lock(overlay_state->mutex);
  overlay_state->camera_roll = msg->position[3];
}

void on_overlay_caps_changed(GstElement *, GstCaps *caps, gpointer user_data) {
  if (caps == nullptr || user_data == nullptr) {
    return;
  }

  GstVideoInfo video_info;
  if (!gst_video_info_from_caps(&video_info, caps)) {
    return;
  }

  auto *overlay_state = static_cast<OverlayState *>(user_data);
  std::scoped_lock<std::mutex> lock(overlay_state->mutex);
  overlay_state->frame_width = static_cast<int>(video_info.width);
  overlay_state->frame_height = static_cast<int>(video_info.height);
}

void on_overlay_draw(GstElement *overlay, cairo_t *cr, guint64, guint64,
                     gpointer user_data) {
  if (cr == nullptr || user_data == nullptr) {
    return;
  }

  auto *overlay_state = static_cast<OverlayState *>(user_data);
  int camera_status = 0;
  int clutch_status = 0;
  bool has_operator_present = false;
  int operator_present_status = 0;
  int frame_width = 0;
  int frame_height = 0;
  double overlay_alpha = 0.7;
  int display_horizontal_offset_px = 0;
  double camera_roll = 0.0;
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
    camera_status =
        overlay_state->camera.present ? overlay_state->camera.get_status() : 0;
    clutch_status =
        overlay_state->clutch.present ? overlay_state->clutch.get_status() : 0;
    has_operator_present = overlay_state->operator_present.present;
    operator_present_status = overlay_state->operator_present.present
                                  ? overlay_state->operator_present.get_status()
                                  : 0;
    frame_width = overlay_state->frame_width;
    frame_height = overlay_state->frame_height;
    overlay_alpha = overlay_state->overlay_alpha;
    display_horizontal_offset_px = overlay_state->display_horizontal_offset_px;
    camera_roll = overlay_state->camera_roll;
    arm_info = overlay_state->arm_info;

    for (const auto &pair : overlay_state->teleop_indicators) {
      const auto &indicator = pair.second;
      if (indicator.is_camera_teleop) {
        has_camera_teleop = true;
        camera_teleop = indicator;
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
  const double eye_width = is_stereo_layout
                               ? static_cast<double>(frame_width) / 2.0
                               : static_cast<double>(frame_width);
  const double image_scale =
      std::min(eye_width, static_cast<double>(frame_height));

  const double horizontal_ui_scale =
      (eye_width > 0) ? (eye_width - 2.0 * std::abs(static_cast<double>(
                                               display_horizontal_offset_px))) /
                            eye_width
                      : 1.0;

  const OverlayTheme theme(image_scale);

  // Draw background strips
  set_source_rgba(cr, theme.bottom_bar_bg, 1.0);
  // Bottom strip (full width)
  cairo_rectangle(cr, 0, frame_height - theme.bottom_bar_height, frame_width,
                  theme.bottom_bar_height);
  cairo_fill(cr);

  const double cy =
      static_cast<double>(frame_height) - theme.bottom_bar_height * 0.5;

  if (is_stereo_layout) {
    const double left_baseline_cx =
        (eye_width / 2.0) -
        static_cast<double>(display_horizontal_offset_px) / 2.0;
    const double left_cx = left_baseline_cx;
    const double pedal_offset = (theme.h_spacing + theme.radius * 2.0) * 0.5;
    draw_status_circle(cr, clutch_status,
                       left_cx - pedal_offset * horizontal_ui_scale, cy,
                       theme.radius, overlay_alpha, theme);
    draw_status_circle(cr, camera_status,
                       left_cx + pedal_offset * horizontal_ui_scale, cy,
                       theme.radius, overlay_alpha, theme);

    const double right_baseline_cx =
        eye_width + (eye_width / 2.0) +
        static_cast<double>(display_horizontal_offset_px) / 2.0;
    const double right_cx = right_baseline_cx;
    draw_status_circle(cr, clutch_status,
                       right_cx - pedal_offset * horizontal_ui_scale, cy,
                       theme.radius, overlay_alpha, theme);
    draw_status_circle(cr, camera_status,
                       right_cx + pedal_offset * horizontal_ui_scale, cy,
                       theme.radius, overlay_alpha, theme);
  } else {
    const double center_cx = static_cast<double>(frame_width) / 2.0;
    const double pedal_offset = (theme.h_spacing + theme.radius * 2.0) * 0.5;
    draw_status_circle(cr, clutch_status, center_cx - pedal_offset, cy,
                       theme.radius, overlay_alpha, theme);
    draw_status_circle(cr, camera_status, center_cx + pedal_offset, cy,
                       theme.radius, overlay_alpha, theme);
  }

  std::sort(left_teleops.begin(), left_teleops.end(),
            [](const TeleopIndicator &a, const TeleopIndicator &b) {
              return a.psm_number < b.psm_number;
            });
  std::sort(right_teleops.begin(), right_teleops.end(),
            [](const TeleopIndicator &a, const TeleopIndicator &b) {
              return a.psm_number < b.psm_number;
            });

  const bool show_op = has_operator_present;
  const bool show_cam = has_camera_teleop;

  if (show_op || show_cam) {
    bool camera_valid = true;
    if (show_cam) {
      camera_valid =
          camera_teleop.arm_name.empty()
              ? true
              : (arm_info.count(camera_teleop.arm_name) > 0
                     ? arm_info[camera_teleop.arm_name].measured_cp_valid
                     : true);
    }

    auto draw_top_icons = [&](const double cx) {
      const double icon_height = theme.radius * 1.6;
      const double cy_top = theme.v_spacing + icon_height * 0.5;
      const double body_width = icon_height * (5.0 / 3.0);
      // Offset from center to achieve h_spacing between rectangular icons
      const double top_offset = (theme.h_spacing + body_width) * 0.5;

      double bg_x_left = cx;
      double bg_width = 0;

      if (show_op && show_cam) {
        bg_x_left = cx - top_offset - body_width * 0.5;
        bg_width = 2.0 * top_offset + body_width;
      } else if (show_op || show_cam) {
        bg_x_left = cx - body_width * 0.5;
        bg_width = body_width;
      }

      const double bg_padding_x = theme.h_spacing;

      set_source_rgba(cr, theme.bottom_bar_bg, 1.0);
      draw_rounded_rectangle(
          cr, bg_x_left - bg_padding_x, 0, bg_width + 2.0 * bg_padding_x,
          theme.v_spacing + icon_height + theme.v_spacing, theme.corner_radius);
      cairo_fill(cr);

      if (show_op && show_cam) {
        draw_operator_present_icon(cr, operator_present_status, cx - top_offset,
                                   cy_top, theme.radius, overlay_alpha, theme);
        draw_camera_icon(cr, camera_teleop.following_active, camera_valid,
                         cx + top_offset, cy_top, theme.radius, overlay_alpha,
                         camera_roll, theme);
      } else if (show_op) {
        draw_operator_present_icon(cr, operator_present_status, cx, cy_top,
                                   theme.radius, overlay_alpha, theme);
      } else if (show_cam) {
        draw_camera_icon(cr, camera_teleop.following_active, camera_valid, cx,
                         cy_top, theme.radius, overlay_alpha, camera_roll,
                         theme);
      }
    };

    if (is_stereo_layout) {
      for (int eye_index = 0; eye_index < 2; ++eye_index) {
        const double offset =
            (eye_index == 0)
                ? -static_cast<double>(display_horizontal_offset_px) / 2.0
                : static_cast<double>(display_horizontal_offset_px) / 2.0;
        const double eye_center_x =
            (eye_width * (static_cast<double>(eye_index) + 0.5)) + offset;
        draw_top_icons(eye_center_x);
      }
    } else {
      draw_top_icons(static_cast<double>(frame_width) / 2.0);
    }
  }

  const auto draw_teleop_column =
      [&](const std::vector<TeleopIndicator> &teleops, const double x,
          const bool label_on_right) {
        for (size_t index = 0; index < teleops.size(); ++index) {
          const double y =
              cy - static_cast<double>(index) * theme.psm_y_step;
          draw_numbered_circle(cr, teleops[index].following_active,
                               arm_info.count(teleops[index].arm_name) > 0
                                   ? arm_info[teleops[index].arm_name]
                                         .measured_cp_valid
                                   : true,
                               teleops[index].psm_number, x, y, theme.radius,
                               overlay_alpha, theme);

          draw_scale_gage(cr, teleops[index].scale, label_on_right, x, y,
                          theme.radius, overlay_alpha, theme);

          // Labels offset: gap + gage width (gage width is 2/3 of diameter)
          const double label_offset = theme.h_spacing + (theme.radius * 2.0) / 3.0;

          const auto info_it = arm_info.find(teleops[index].arm_name);
          if (info_it != arm_info.end()) {
            draw_tool_type_label(cr, info_it->second.tool_type, label_on_right,
                                 x, y, theme.radius, overlay_alpha,
                                 label_offset, theme);
          }

          draw_scale_label(cr, teleops[index].current_state, label_on_right, x,
                           y, theme.radius, overlay_alpha, label_offset, theme);
        }
      };

  if (is_stereo_layout) {
    for (int eye_index = 0; eye_index < 2; ++eye_index) {
      const double eye_x_offset = eye_width * static_cast<double>(eye_index);
      const double shift =
          (eye_index == 0)
              ? -static_cast<double>(display_horizontal_offset_px) / 2.0
              : static_cast<double>(display_horizontal_offset_px) / 2.0;

      const double psm_left_x = eye_x_offset + (eye_width / 2.0) + shift -
                                (eye_width / 2.0) * horizontal_ui_scale +
                                theme.psm_x_margin * horizontal_ui_scale;
      const double psm_right_x = eye_x_offset + (eye_width / 2.0) + shift +
                                 (eye_width / 2.0) * horizontal_ui_scale -
                                 theme.psm_x_margin * horizontal_ui_scale;
      draw_teleop_column(left_teleops, psm_left_x, true);
      draw_teleop_column(right_teleops, psm_right_x, false);
    }
  } else {
    // Single-eye layout: both PSM columns shown in every eye, only corner dot
    // differs.
    constexpr double k_eye_dot_radius_ratio = 2.0 / 425.0;
    const double dot_r = image_scale * k_eye_dot_radius_ratio;
    const double dot_x = overlay_view == OverlayView::LeftEye
                             ? dot_r
                             : static_cast<double>(frame_width) - dot_r;
    cairo_new_path(cr);
    cairo_arc(cr, dot_x, dot_r, dot_r, 0.0, 2.0 * M_PI);
    cairo_set_source_rgba(cr, 0.82, 0.82, 0.82, overlay_alpha);
    cairo_fill(cr);
    draw_teleop_column(left_teleops, theme.psm_x_margin, true);
    draw_teleop_column(right_teleops,
                       static_cast<double>(frame_width) - theme.psm_x_margin,
                       false);
  }
}

} // namespace sv

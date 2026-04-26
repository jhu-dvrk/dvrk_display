#include "overlay_components.hpp"
#include "overlay_utils.hpp"
#include <cmath>
#include <algorithm>

namespace sv {

void draw_status_circle(cairo_t *cr, int status, double cx, double cy,
                        double radius, double alpha,
                        const OverlayTheme &theme) {
  cairo_new_path(cr);
  cairo_move_to(cr, cx - radius, cy);
  cairo_arc(cr, cx, cy, radius, M_PI, 0.0);
  cairo_line_to(cr, cx + radius, cy + radius - theme.corner_radius);
  cairo_arc(cr, cx + radius - theme.corner_radius,
            cy + radius - theme.corner_radius, theme.corner_radius, 0.0,
            0.5 * M_PI);
  cairo_line_to(cr, cx - radius + theme.corner_radius, cy + radius);
  cairo_arc(cr, cx - radius + theme.corner_radius,
            cy + radius - theme.corner_radius, theme.corner_radius, 0.5 * M_PI,
            M_PI);
  cairo_close_path(cr);

  if (status != 0) {
    if (status == 2) {
      set_source_rgba(cr, theme.active_green, alpha);
    } else {
      set_source_rgba(cr, theme.active_grey, alpha);
    }
    cairo_fill_preserve(cr);
  }

  set_source_rgba(cr, theme.valid_grey, alpha);
  cairo_set_line_width(cr, theme.line_width);
  cairo_stroke(cr);
}

void draw_numbered_circle(cairo_t *cr, bool active, bool valid, int number,
                          double cx, double cy, double radius, double alpha,
                          const OverlayTheme &theme) {
  cairo_new_path(cr);
  cairo_arc(cr, cx, cy, radius, 0.0, 2.0 * M_PI);
  if (active) {
    if (valid) {
      set_source_rgba(cr, theme.active_grey, alpha);
    } else {
      set_source_rgba(cr, theme.invalid_red, alpha);
    }
    cairo_fill_preserve(cr);
  }

  if (valid) {
    set_source_rgba(cr, theme.valid_grey, alpha);
  } else {
    set_source_rgba(cr, theme.invalid_red, alpha);
  }
  cairo_set_line_width(cr, theme.line_width);
  cairo_stroke(cr);

  const std::string label = std::to_string(number);
  cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
                         CAIRO_FONT_WEIGHT_BOLD);
  cairo_set_font_size(cr, radius * 1.1);

  cairo_text_extents_t extents;
  cairo_text_extents(cr, label.c_str(), &extents);
  cairo_move_to(cr, cx - (extents.width / 2.0 + extents.x_bearing),
                cy - (extents.height / 2.0 + extents.y_bearing));
  set_source_rgba(cr, theme.text_light, alpha);
  cairo_show_text(cr, label.c_str());
}

void draw_scale_gage(cairo_t *cr, double scale, bool on_right, double cx,
                     double cy, double radius, double alpha,
                     const OverlayTheme &theme) {
  const double gage_height = radius * 2.0;
  const double gage_width = gage_height / 3.0;

  const double x = on_right ? (cx + radius + theme.h_spacing)
                            : (cx - radius - theme.h_spacing - gage_width);
  const double y = cy - radius;

  // Background
  draw_rounded_rectangle(cr, x, y, gage_width, gage_height, theme.corner_radius);
  set_source_rgba(cr, theme.valid_grey, alpha);
  cairo_set_line_width(cr, theme.line_width);
  cairo_stroke(cr);

  // Fill
  const double clamped_scale = std::clamp(scale, 0.0, 1.0);
  const double fill_height = gage_height * clamped_scale;
  if (fill_height > 0) {
    draw_rounded_rectangle(cr, x, y + gage_height - fill_height, gage_width,
                           fill_height, theme.corner_radius);
    set_source_rgba(cr, theme.active_grey, alpha);
    cairo_fill(cr);
  }
}

void draw_tool_type_label(cairo_t *cr, const std::string &tool_type,
                          bool left_side, double cx, double cy, double radius,
                          double alpha, double extra_offset,
                          const OverlayTheme &theme) {
  const std::string display_tool_type = format_tool_type_label(tool_type);
  if (display_tool_type.empty()) {
    return;
  }

  cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
                         CAIRO_FONT_WEIGHT_BOLD);
  cairo_set_font_size(cr, theme.text_height);

  cairo_text_extents_t extents;
  cairo_text_extents(cr, display_tool_type.c_str(), &extents);

  const double text_x = left_side
                            ? (cx + radius + theme.h_spacing + extra_offset)
                            : (cx - radius - theme.h_spacing - extents.width -
                               extents.x_bearing - extra_offset);
  // Position text top at cy + v_spacing*0.5
  const double text_y = cy + theme.v_spacing * 0.5 - extents.y_bearing;

  cairo_move_to(cr, text_x, text_y);
  set_source_rgba(cr, theme.text_light, alpha);
  cairo_show_text(cr, display_tool_type.c_str());
}

void draw_scale_label(cairo_t *cr, const std::string &state, bool left_side,
                      double cx, double cy, double radius, double alpha,
                      double extra_offset, const OverlayTheme &theme) {
  std::string label = "";
  if (state == "DISABLED") {
    label = "Disabled";
  } else if (state == "ALIGNING_MTM") {
    label = "Aligning...";
  } else if (state == "SETTING_ARMS_STATE") {
    label = "Checking arms";
  }

  if (label.empty()) {
    return;
  }

  cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
                         CAIRO_FONT_WEIGHT_BOLD);
  cairo_set_font_size(cr, theme.text_height);

  cairo_text_extents_t extents;
  cairo_text_extents(cr, label.c_str(), &extents);

  const double text_x = left_side
                            ? (cx + radius + theme.h_spacing + extra_offset)
                            : (cx - radius - theme.h_spacing - extents.width -
                               extents.x_bearing - extra_offset);
  // Position text bottom at cy - v_spacing*0.5
  const double text_y =
      cy - theme.v_spacing * 0.5 - (extents.height + extents.y_bearing);

  cairo_move_to(cr, text_x, text_y);
  set_source_rgba(cr, theme.text_light, alpha);
  cairo_show_text(cr, label.c_str());
}

void draw_camera_icon(cairo_t *cr, bool active, bool valid, double cx, double cy,
                      double radius, double alpha, const OverlayTheme &theme) {
  const RgbaColor outline_color =
      valid ? theme.valid_grey : theme.invalid_red;

  const double body_height = radius * 1.6;
  const double body_width =
      body_height * (5.0 / 3.0); // 5:3 horizontal rectangle
  const double x_left = cx - body_width * 0.5;
  const double y_top = cy - body_height * 0.5;

  cairo_new_path(cr);
  cairo_move_to(cr, x_left + theme.corner_radius, y_top);
  cairo_line_to(cr, x_left + body_width - theme.corner_radius, y_top);
  cairo_arc(cr, x_left + body_width - theme.corner_radius,
            y_top + theme.corner_radius, theme.corner_radius, -0.5 * M_PI, 0.0);
  cairo_line_to(cr, x_left + body_width,
                y_top + body_height - theme.corner_radius);
  cairo_arc(cr, x_left + body_width - theme.corner_radius,
            y_top + body_height - theme.corner_radius, theme.corner_radius, 0.0,
            0.5 * M_PI);
  cairo_line_to(cr, x_left + theme.corner_radius, y_top + body_height);
  cairo_arc(cr, x_left + theme.corner_radius,
            y_top + body_height - theme.corner_radius, theme.corner_radius,
            0.5 * M_PI, M_PI);
  cairo_line_to(cr, x_left, y_top + theme.corner_radius);
  cairo_arc(cr, x_left + theme.corner_radius, y_top + theme.corner_radius,
            theme.corner_radius, M_PI, 1.5 * M_PI);
  cairo_close_path(cr);

  if (active) {
    set_source_rgba(cr, theme.active_grey, alpha);
    cairo_fill_preserve(cr);
  }

  set_source_rgba(cr, outline_color, alpha);
  cairo_set_line_width(cr, theme.line_width);
  cairo_stroke(cr);

  cairo_new_path(cr);
  cairo_arc(cr, cx, cy, body_height * 0.45, 0.0, 2.0 * M_PI);
  set_source_rgba(cr, outline_color, alpha);
  cairo_set_line_width(cr, theme.line_width);
  cairo_stroke(cr);
}

void draw_operator_present_icon(cairo_t *cr, int status, double cx, double cy,
                                double radius, double alpha,
                                const OverlayTheme &theme) {
  const RgbaColor outline_color = theme.valid_grey;

  const double body_height = radius * 1.6;
  const double body_width = body_height * (5.0 / 3.0);
  const double x_left = cx - body_width * 0.5;
  const double y_top = cy - body_height * 0.5;

  draw_rounded_rectangle(cr, x_left, y_top, body_width, body_height,
                         theme.corner_radius);

  if (status != 0) {
    if (status == 2) {
      set_source_rgba(cr, theme.active_green, alpha);
    } else {
      set_source_rgba(cr, theme.active_grey, alpha);
    }
    cairo_fill_preserve(cr);
  }

  set_source_rgba(cr, outline_color, alpha);
  cairo_set_line_width(cr, theme.line_width);
  cairo_stroke(cr);

  cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL,
                         CAIRO_FONT_WEIGHT_BOLD);
  cairo_set_font_size(cr, radius * 0.9);
  cairo_text_extents_t extents;
  cairo_text_extents(cr, "OP", &extents);
  cairo_move_to(cr, cx - (extents.width / 2.0 + extents.x_bearing),
                cy - (extents.height / 2.0 + extents.y_bearing));
  if (status != 0) {
    set_source_rgba(cr, theme.text_dark, alpha);
  } else {
    set_source_rgba(cr, outline_color, alpha);
  }
  cairo_show_text(cr, "OP");
}

} // namespace sv

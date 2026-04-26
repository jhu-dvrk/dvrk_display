#include "overlay_utils.hpp"
#include <algorithm>
#include <cmath>

namespace sv {

void set_source_rgba(cairo_t *cr, const RgbaColor &color, double alpha) {
  cairo_set_source_rgba(cr, color.r, color.g, color.b, alpha * color.a_mult);
}

void draw_rounded_rectangle(cairo_t *cr, double x, double y, double width,
                            double height, double corner_radius) {
  cairo_new_path(cr);
  cairo_move_to(cr, x + corner_radius, y);
  cairo_line_to(cr, x + width - corner_radius, y);
  cairo_arc(cr, x + width - corner_radius, y + corner_radius, corner_radius,
            -0.5 * M_PI, 0.0);
  cairo_line_to(cr, x + width, y + height - corner_radius);
  cairo_arc(cr, x + width - corner_radius, y + height - corner_radius,
            corner_radius, 0.0, 0.5 * M_PI);
  cairo_line_to(cr, x + corner_radius, y + height);
  cairo_arc(cr, x + corner_radius, y + height - corner_radius, corner_radius,
            0.5 * M_PI, M_PI);
  cairo_line_to(cr, x, y + corner_radius);
  cairo_arc(cr, x + corner_radius, y + corner_radius, corner_radius, M_PI,
            1.5 * M_PI);
  cairo_close_path(cr);
}

std::string format_tool_type_label(const std::string &raw_tool_type) {
  if (raw_tool_type.empty()) {
    return "";
  }

  auto colon_pos = raw_tool_type.find(':');
  std::string formatted;
  if (colon_pos != std::string::npos) {
    formatted = raw_tool_type.substr(0, colon_pos);
  } else {
    formatted = raw_tool_type;
  }

  if (formatted.empty()) {
    return "";
  }

  std::replace(formatted.begin(), formatted.end(), '_', ' ');

  bool capitalize_next = true;
  for (auto &c : formatted) {
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

} // namespace sv

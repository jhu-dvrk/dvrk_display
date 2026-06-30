#ifndef SV_OVERLAY_UTILS_HPP
#define SV_OVERLAY_UTILS_HPP

#include "overlay_theme.hpp"
#include <cairo/cairo.h>
#include <string>

namespace sv {

void set_source_rgba(cairo_t *cr, const RgbaColor &color, double alpha);

void draw_rounded_rectangle(cairo_t *cr, double x, double y, double width,
                            double height, double corner_radius);

std::string format_tool_type_label(const std::string &raw_tool_type);

} // namespace sv

#endif // SV_OVERLAY_UTILS_HPP

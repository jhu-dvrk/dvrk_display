#ifndef SV_OVERLAY_COMPONENTS_HPP
#define SV_OVERLAY_COMPONENTS_HPP

#include "overlay_theme.hpp"
#include <cairo/cairo.h>
#include <string>

namespace sv {

void draw_status_circle(cairo_t *cr, int status, double cx, double cy,
                        double radius, double alpha, const OverlayTheme &theme);

void draw_numbered_circle(cairo_t *cr, bool active, bool valid, int number,
                          double cx, double cy, double radius, double alpha,
                          const OverlayTheme &theme);

void draw_scale_gage(cairo_t *cr, double scale, bool on_right, double cx,
                     double cy, double radius, double alpha,
                     const OverlayTheme &theme);

void draw_tool_type_label(cairo_t *cr, const std::string &tool_type,
                          bool left_side, double cx, double cy, double radius,
                          double alpha, double extra_offset,
                          const OverlayTheme &theme);

void draw_scale_label(cairo_t *cr, const std::string &state, bool left_side,
                      double cx, double cy, double radius, double alpha,
                      double extra_offset, const OverlayTheme &theme);

void draw_camera_icon(cairo_t *cr, bool active, bool valid, double cx, double cy,
                      double radius, double alpha, double roll,
                      const OverlayTheme &theme);

void draw_operator_present_icon(cairo_t *cr, int status, double cx, double cy,
                                double radius, double alpha,
                                const OverlayTheme &theme);

} // namespace sv

#endif // SV_OVERLAY_COMPONENTS_HPP

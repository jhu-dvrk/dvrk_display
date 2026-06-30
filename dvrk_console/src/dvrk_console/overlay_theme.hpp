#ifndef SV_OVERLAY_THEME_HPP
#define SV_OVERLAY_THEME_HPP

namespace sv {

struct RgbaColor {
  double r, g, b, a_mult;
};

struct OverlayTheme {
  // Colors
  static constexpr RgbaColor active_green = {0.2, 0.8, 0.2, 1.0};
  static constexpr RgbaColor active_grey = {0.75, 0.75, 0.75, 1.0};
  static constexpr RgbaColor valid_grey = {0.82, 0.82, 0.82, 1.0};
  static constexpr RgbaColor invalid_red = {0.9, 0.15, 0.15, 1.0};
  static constexpr RgbaColor text_light = {0.95, 0.95, 0.95, 1.0};
  static constexpr RgbaColor text_dark = {0.1, 0.1, 0.1, 1.0};
  static constexpr RgbaColor bottom_bar_bg = {0.1, 0.1, 0.1, 0.4};

  // Dimensions
  static constexpr double line_width = 2.0;

  // Absolute dimensions (scaled from ratios)
  double h_spacing;
  double v_spacing;
  double radius;
  double corner_radius;
  double text_height;
  double bottom_bar_height;
  double psm_x_margin;
  double psm_y_step;

  OverlayTheme(double image_scale) {
    // These ratios match the current design exactly
    constexpr double horizontal_spacing_ratio = 6.0 / 425.0;
    constexpr double vertical_spacing_ratio = 3.0 / 425.0;
    constexpr double psm_radius_ratio = 7.0 / 425.0;
    constexpr double corner_radius_ratio = 1.8 / 425.0;
    constexpr double psm_x_margin_ratio = 30.0 / 425.0;

    h_spacing = image_scale * horizontal_spacing_ratio;
    v_spacing = image_scale * vertical_spacing_ratio;
    radius = image_scale * psm_radius_ratio;
    corner_radius = image_scale * corner_radius_ratio;
    text_height = radius * 0.8;
    bottom_bar_height = 3.0 * v_spacing + 2.0 * text_height;
    psm_x_margin = image_scale * psm_x_margin_ratio;
    psm_y_step = v_spacing + text_height;
  }
};

} // namespace sv

#endif // SV_OVERLAY_THEME_HPP

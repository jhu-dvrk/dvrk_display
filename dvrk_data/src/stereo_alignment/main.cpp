#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include <glib-unix.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <dvrk_data/config.hpp>
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <dvrk_data/stereo_common.hpp>

namespace {

struct CommandLineOptions {
  std::string config_file;
};

struct CropValues {
  int left = 0;
  int right = 0;
  int top = 0;
  int bottom = 0;
};

struct FrameTimestampState {
  std::mutex mutex;
  gint64 left_source_ts = 0;
  gint64 right_source_ts = 0;
};

void print_usage(const char *executable) {
  std::cerr << "Usage: " << executable << " -c <config.json>" << std::endl;
}

bool parse_arguments(int argc, char *argv[], CommandLineOptions &options) {
  bool seen_config = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ros-args") {
      break;
    }

    if (arg == "-c" && i + 1 < argc) {
      if (seen_config) {
        std::cerr << "Error: multiple -c arguments are not supported."
                  << std::endl;
        return false;
      }
      options.config_file = argv[++i];
      seen_config = true;
      continue;
    }

    std::cerr << "Error: unknown argument '" << arg << "'." << std::endl;
    return false;
  }

  if (!seen_config) {
    std::cerr << "Error: exactly one config file is required." << std::endl;
    return false;
  }

  return true;
}

int clip_int(const int value, const int min_value, const int max_value) {
  return std::max(min_value, std::min(max_value, value));
}

std::pair<int, int> offset_valid_range(const int working_size,
                                       const int eye_size) {
  const int crop_total = std::max(0, working_size - eye_size);
  const int center = crop_total / 2;

  const int min_half = -center;
  const int max_half = crop_total - center;

  const int min_offset =
      static_cast<int>(std::ceil(2.0 * static_cast<double>(min_half))) + 1;
  const int max_offset =
      static_cast<int>(std::floor(2.0 * static_cast<double>(max_half))) - 1;

  if (min_offset > max_offset) {
    return {0, 0};
  }
  return {min_offset, max_offset};
}

int clamp_offset_to_valid(const int working_size, const int eye_size,
                           const int offset_px) {
  const auto [min_offset, max_offset] =
      offset_valid_range(working_size, eye_size);
  return clip_int(offset_px, min_offset, max_offset);
}

int normalize_eye_size_for_even_crop(const int working_size,
                                     const int requested_eye_size) {
  if (working_size <= 0) {
    return std::max(1, requested_eye_size);
  }

  int eye_size = clip_int(requested_eye_size, 1, working_size);
  if (((working_size - eye_size) & 1) != 0) {
    if (eye_size > 1) {
      --eye_size;
    } else if (eye_size < working_size) {
      ++eye_size;
    }
  }
  return eye_size;
}

std::pair<int, int> compute_axis_starts(const int crop_total,
                                        const int offset_px) {
  const int center = crop_total / 2;
  const int negative_start =
      center -
      static_cast<int>(std::floor(static_cast<double>(offset_px) / 2.0));
  const int positive_start =
      center +
      static_cast<int>(std::ceil(static_cast<double>(offset_px) / 2.0));
  return {negative_start, positive_start};
}

CropValues compute_eye_crop(const int working_w, const int working_h,
                             const int eye_w, const int eye_h,
                             const int baseline_px,
                             const int vertical_offset_px, const int sign) {
  const int crop_x_total = std::max(0, working_w - eye_w);
  const int crop_y_total = std::max(0, working_h - eye_h);

  const auto [left_start, right_start] =
      compute_axis_starts(crop_x_total, baseline_px);
  const auto [top_start, bottom_start] =
      compute_axis_starts(crop_y_total, vertical_offset_px);

  CropValues crop;
  crop.left = (sign < 0 ? left_start : right_start) & ~1;
  crop.right = (crop_x_total - crop.left) & ~1;
  crop.top = (sign < 0 ? top_start : bottom_start) & ~1;
  crop.bottom = (crop_y_total - crop.top) & ~1;
  return crop;
}

GstPadProbeReturn input_timestamp_probe_cb(GstPad *pad,
                                           GstPadProbeInfo *info,
                                           gpointer user_data) {
  (void)pad;
  if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER)) {
    return GST_PAD_PROBE_OK;
  }

  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
  if (!gst_buffer_is_writable(buf)) {
    buf = gst_buffer_make_writable(buf);
    GST_PAD_PROBE_INFO_DATA(info) = buf;
  }

  auto *state = static_cast<FrameTimestampState *>(user_data);
  if (state == nullptr) {
    return GST_PAD_PROBE_OK;
  }

  std::string element_name;
  GstElement *parent = gst_pad_get_parent_element(pad);
  if (parent != nullptr) {
    const char *name = GST_OBJECT_NAME(parent);
    if (name != nullptr) {
      element_name = name;
    }
    gst_object_unref(parent);
  }

  DcFrameTimestamps timestamps = dc_buffer_get_frame_timestamps(buf);
  const gint64 now = dc_clock_realtime_ns();

  std::scoped_lock<std::mutex> lock(state->mutex);
  if (element_name == "__left_src_q__") {
    if (timestamps.left_source_ts == 0) {
      timestamps.left_source_ts = now;
    }
    state->left_source_ts = timestamps.left_source_ts;
  } else if (element_name == "__right_src_q__") {
    if (timestamps.right_source_ts == 0) {
      timestamps.right_source_ts = now;
    }
    state->right_source_ts = timestamps.right_source_ts;
  }

  dc_buffer_set_frame_timestamps(buf, timestamps);
  return GST_PAD_PROBE_OK;
}

GstPadProbeReturn output_timestamp_probe_cb(GstPad *pad,
                                            GstPadProbeInfo *info,
                                            gpointer user_data) {
  (void)pad;
  if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER)) {
    return GST_PAD_PROBE_OK;
  }

  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
  if (!gst_buffer_is_writable(buf)) {
    buf = gst_buffer_make_writable(buf);
    GST_PAD_PROBE_INFO_DATA(info) = buf;
  }

  auto *state = static_cast<FrameTimestampState *>(user_data);
  if (state == nullptr) {
    return GST_PAD_PROBE_OK;
  }

  DcFrameTimestamps timestamps = dc_buffer_get_frame_timestamps(buf);
  {
    std::scoped_lock<std::mutex> lock(state->mutex);
    if (timestamps.left_source_ts == 0) {
      timestamps.left_source_ts = state->left_source_ts;
    }
    if (timestamps.right_source_ts == 0) {
      timestamps.right_source_ts = state->right_source_ts;
    }
  }
  timestamps.stereo_output_ts = dc_clock_realtime_ns();
  dc_buffer_set_frame_timestamps(buf, timestamps);
  return GST_PAD_PROBE_OK;
}

void add_probe(GstElement *pipeline, const std::string &element_name,
               GstPadProbeCallback callback, FrameTimestampState *state) {
  GstElement *element =
      gst_bin_get_by_name(GST_BIN(pipeline), element_name.c_str());
  if (element == nullptr) {
    return;
  }

  GstPad *pad = gst_element_get_static_pad(element, "src");
  if (pad != nullptr) {
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, callback, state, nullptr);
    gst_object_unref(pad);
  }
  gst_object_unref(element);
}

std::string build_pipeline_string(
    const sv::AppConfig &cfg,
    const std::vector<sv::UnixfdSinkConfig> &stereo_sinks) {
  int base_crop_w = cfg.crop_width > 0 ? cfg.crop_width : cfg.original_width;
  int base_crop_h = cfg.crop_height > 0 ? cfg.crop_height : cfg.original_height;
  base_crop_w = normalize_eye_size_for_even_crop(cfg.original_width, base_crop_w);
  base_crop_h =
      normalize_eye_size_for_even_crop(cfg.original_height, base_crop_h);

  int aspect_crop_l = 0;
  int aspect_crop_r = 0;
  int aspect_crop_t = 0;
  int aspect_crop_b = 0;

  if (cfg.preserve_size && cfg.original_width > 0 && cfg.original_height > 0) {
    const double orig_aspect = static_cast<double>(cfg.original_width) /
                               static_cast<double>(cfg.original_height);
    int w_c_prime = std::min(
        base_crop_w, static_cast<int>(std::round(base_crop_h * orig_aspect)));
    int h_c_prime = std::min(
        base_crop_h, static_cast<int>(std::round(base_crop_w / orig_aspect)));

    int diff_x = base_crop_w - w_c_prime;
    int diff_y = base_crop_h - h_c_prime;
    diff_x = (diff_x + 1) & ~1;
    diff_y = (diff_y + 1) & ~1;

    aspect_crop_l = diff_x / 2;
    aspect_crop_r = diff_x - aspect_crop_l;
    aspect_crop_t = diff_y / 2;
    aspect_crop_b = diff_y - aspect_crop_t;
  }

  const int eye_w = cfg.preserve_size ? cfg.original_width : base_crop_w;
  const int eye_h = cfg.preserve_size ? cfg.original_height : base_crop_h;

  const int horizontal_shift_px = clamp_offset_to_valid(
      cfg.original_width, base_crop_w, cfg.horizontal_shift_px);
  const int vertical_shift_px = clamp_offset_to_valid(
      cfg.original_height, base_crop_h, cfg.vertical_shift_px);

  CropValues left_crop =
      compute_eye_crop(cfg.original_width, cfg.original_height, base_crop_w,
                       base_crop_h, horizontal_shift_px, vertical_shift_px, -1);
  CropValues right_crop =
      compute_eye_crop(cfg.original_width, cfg.original_height, base_crop_w,
                       base_crop_h, horizontal_shift_px, vertical_shift_px, 1);

  if (cfg.preserve_size) {
    left_crop.left += aspect_crop_l;
    left_crop.right += aspect_crop_r;
    left_crop.top += aspect_crop_t;
    left_crop.bottom += aspect_crop_b;

    right_crop.left += aspect_crop_l;
    right_crop.right += aspect_crop_r;
    right_crop.top += aspect_crop_t;
    right_crop.bottom += aspect_crop_b;
  }

  std::string scale_suffix;
  if (cfg.preserve_size && cfg.original_width > 0 && cfg.original_height > 0) {
    scale_suffix = " ! videoscale ! video/x-raw,width=" +
                   std::to_string(cfg.original_width) +
                   ",height=" + std::to_string(cfg.original_height);
  }

  const std::string left_chain =
      cfg.left.source +
      " ! queue name=__left_src_q__ max-size-buffers=3 max-size-time=0 "
      "max-size-bytes=0 leaky=downstream"
      " ! videobalance name=left_balance brightness=" + std::to_string(cfg.left_color.brightness) +
      " contrast=" + std::to_string(cfg.left_color.contrast) +
      " saturation=" + std::to_string(cfg.left_color.saturation) +
      " hue=" + std::to_string(cfg.left_color.hue) +
      " ! videocrop name=left_crop left=" + std::to_string(left_crop.left) +
      " right=" + std::to_string(left_crop.right) +
      " top=" + std::to_string(left_crop.top) +
      " bottom=" + std::to_string(left_crop.bottom) + scale_suffix +
      " ! videoconvert ! video/x-raw,format=I420,width=" +
      std::to_string(eye_w) + ",height=" + std::to_string(eye_h) +
      " ! __stereo_mix__.sink_0";

  const std::string right_chain =
      cfg.right.source +
      " ! queue name=__right_src_q__ max-size-buffers=3 max-size-time=0 "
      "max-size-bytes=0 leaky=downstream"
      " ! videobalance name=right_balance brightness=" + std::to_string(cfg.right_color.brightness) +
      " contrast=" + std::to_string(cfg.right_color.contrast) +
      " saturation=" + std::to_string(cfg.right_color.saturation) +
      " hue=" + std::to_string(cfg.right_color.hue) +
      " ! videocrop name=right_crop left=" + std::to_string(right_crop.left) +
      " right=" + std::to_string(right_crop.right) +
      " top=" + std::to_string(right_crop.top) +
      " bottom=" + std::to_string(right_crop.bottom) + scale_suffix +
      " ! videoconvert ! video/x-raw,format=I420,width=" +
      std::to_string(eye_w) + ",height=" + std::to_string(eye_h) +
      " ! __stereo_mix__.sink_1";

  std::string output_chain =
      "compositor name=__stereo_mix__ background=1"
      " sink_0::xpos=0 sink_0::ypos=0"
      " sink_0::width=" +
      std::to_string(eye_w) + " sink_0::height=" + std::to_string(eye_h) +
      " sink_1::xpos=" + std::to_string(eye_w) +
      " sink_1::ypos=0"
      " sink_1::width=" +
      std::to_string(eye_w) + " sink_1::height=" + std::to_string(eye_h) +
      " ! video/x-raw,format=I420,width=" + std::to_string(2 * eye_w) +
      ",height=" + std::to_string(eye_h);

  std::string outputs;
  if (stereo_sinks.size() > 1) {
    outputs += " ! tee name=__stereo_out__ ";
    for (std::size_t i = 0; i < stereo_sinks.size(); ++i) {
      const std::string socket_path =
          dc_stereo::resolve_unixfd_socket_path(cfg.name, stereo_sinks[i]);
      outputs += " __stereo_out__. ! queue name=__stereo_unixfd_ts_q" + std::to_string(i) +
                 "__ max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
                 "leaky=downstream ! unixfdsink socket-path=" +
                 socket_path + " socket-type=abstract sync=false async=false";
    }
  } else if (stereo_sinks.size() == 1) {
    const std::string socket_path =
        dc_stereo::resolve_unixfd_socket_path(cfg.name, stereo_sinks[0]);
    outputs += " ! queue name=__stereo_unixfd_ts_q0__ max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
               "leaky=downstream ! unixfdsink socket-path=" +
               socket_path + " socket-type=abstract sync=false async=false";
  }

  return left_chain + " " + right_chain + " " + output_chain + " " + outputs;
}

}  // namespace

int main(int argc, char *argv[]) {
  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);
  dc_frame_timestamps_meta_register();

  CommandLineOptions options;
  if (!parse_arguments(argc, argv, options)) {
    print_usage(argv[0]);
    rclcpp::shutdown();
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("stereo_alignment");
  const std::string &path = options.config_file;
  if (!std::filesystem::exists(path)) {
    RCLCPP_ERROR(node->get_logger(), "Config file does not exist: %s",
                 path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  Json::Value root;
  if (!sv::Config::load_from_file(path, root)) {
    rclcpp::shutdown();
    return 1;
  }

  if (!sv::Config::check_type(root, "dvrk_data:stereo_alignment@1.0.0",
                              path)) {
    rclcpp::shutdown();
    return 1;
  }

  sv::AppConfig cfg;
  try {
    cfg = sv::Config::parse_app_config(root);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  if (cfg.left.source.empty()) {
    auto srcs = dc_stereo::collect_unixfd_sources(cfg, "left");
    if (!srcs.empty()) {
      const std::string path =
          dc_stereo::resolve_unixfd_source_path(cfg.name, srcs[0]);
      cfg.left.source =
          dc_stereo::build_unixfdsrc_string(path, cfg.original_width,
                                            cfg.original_height);
      RCLCPP_INFO(node->get_logger(), "left unixfd source: %s", path.c_str());
    }
  }
  if (cfg.right.source.empty()) {
    auto srcs = dc_stereo::collect_unixfd_sources(cfg, "right");
    if (!srcs.empty()) {
      const std::string path =
          dc_stereo::resolve_unixfd_source_path(cfg.name, srcs[0]);
      cfg.right.source =
          dc_stereo::build_unixfdsrc_string(path, cfg.original_width,
                                            cfg.original_height);
      RCLCPP_INFO(node->get_logger(), "right unixfd source: %s", path.c_str());
    }
  }

  if (cfg.left.source.empty() || cfg.right.source.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Config '%s' must define camera.left.stream and "
                 "camera.right.stream",
                 cfg.name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  if (cfg.original_width <= 0 || cfg.original_height <= 0 ||
      cfg.crop_width <= 0 || cfg.crop_height <= 0) {
    RCLCPP_ERROR(node->get_logger(),
                 "Config '%s' must provide positive camera.size and crop "
                 "dimensions",
                 cfg.name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  auto stereo_sinks = dc_stereo::collect_unixfd_sinks(cfg, "stereo");
  dc_stereo::ensure_sink(stereo_sinks, "stereo");
  dc_stereo::remove_stale_sockets(cfg.name, stereo_sinks, node->get_logger());

  for (const auto &sink : stereo_sinks) {
    RCLCPP_INFO(node->get_logger(), "stereo unixfd sink: %s",
                dc_stereo::resolve_unixfd_socket_path(cfg.name, sink).c_str());
  }

  const std::string pipeline_string =
      build_pipeline_string(cfg, stereo_sinks);
  if (!dc_stereo::validate_pipeline(pipeline_string, node->get_logger(),
                                    "stereo_alignment")) {
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "GStreamer pipeline string:\n%s",
              pipeline_string.c_str());

  GError *error = nullptr;
  GstElement *pipeline = gst_parse_launch(pipeline_string.c_str(), &error);
  if (error != nullptr || pipeline == nullptr) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create pipeline: %s",
                 error && error->message ? error->message : "unknown");
    if (error != nullptr) {
      g_error_free(error);
    }
    if (pipeline != nullptr) {
      gst_object_unref(pipeline);
    }
    rclcpp::shutdown();
    return 1;
  }

  FrameTimestampState timestamp_state;
  add_probe(pipeline, "__left_src_q__", input_timestamp_probe_cb,
            &timestamp_state);
  add_probe(pipeline, "__right_src_q__", input_timestamp_probe_cb,
            &timestamp_state);
  for (std::size_t i = 0; i < stereo_sinks.size(); ++i) {
    add_probe(pipeline, "__stereo_unixfd_ts_q" + std::to_string(i) + "__",
              output_timestamp_probe_cb, &timestamp_state);
  }

  const int status = dc_stereo::run_pipeline(
      pipeline, node, "Stereo alignment background pipeline started");
  gst_object_unref(pipeline);
  rclcpp::shutdown();
  return status;
}

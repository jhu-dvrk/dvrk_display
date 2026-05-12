#include <geometry_msgs/msg/pose_stamped.hpp>
#include <glib-unix.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <gst/video/navigation.h>
#include <gtkmm.h>
#include <gdk/gdkkeysyms.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <pwd.h>
#include <string>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "config.hpp"
#include "display_output_panel.hpp"
#include <data_collection/gst_utils.hpp>
#include "overlay.hpp"
#include <data_collection/cpu_timestamp_meta.hpp>

namespace {

struct CommandLineOptions {
  std::string config_file;
  bool show_grid = false;
  bool dump_dot = false;
  GstDebugGraphDetails dot_flags = GST_DEBUG_GRAPH_SHOW_ALL;
};

struct CropValues {
  int left = 0;
  int right = 0;
  int top = 0;
  int bottom = 0;
};

static Glib::RefPtr<Gtk::Application> g_app;

struct FrameTimestampState {
  std::mutex mutex;
  gint64 left_source_ts = 0;
  gint64 right_source_ts = 0;
  gint64 stereo_output_ts = 0;
};

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
                            const int baseline_px, const int vertical_offset_px,
                            const int sign) {
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

void print_usage(const char *executable) {
  std::cerr << "Usage: " << executable
            << " -c <config.json> [--grid] [-g <0|1|2|3>]" << std::endl;
  std::cerr << "  --grid   Display calibration grid overlay for display alignment"
            << std::endl;
  dc::print_dot_usage();
}

bool parse_arguments(int argc, char *argv[], CommandLineOptions &options) {
  bool seen_config = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-c" && i + 1 < argc) {
      if (seen_config) {
        std::cerr << "Error: multiple -c arguments are not supported; provide "
                     "exactly one config file."
                  << std::endl;
        return false;
      }
      options.config_file = argv[++i];
      seen_config = true;
      continue;
    }

    if (arg == "--grid") {
      options.show_grid = true;
      continue;
    }

    if (dc::parse_dot_arguments(i, argc, argv, options.dump_dot, options.dot_flags)) {
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

bool validate_pipeline(const std::string &stream, const rclcpp::Logger &logger,
                       const std::string &name) {
  if (stream.empty()) {
    RCLCPP_WARN(logger,
                "Video entry '%s' does not define a GStreamer stream yet",
                name.c_str());
    return false;
  }

  const std::vector<std::string> candidates = {stream, stream + " ! fakesink"};

  for (const auto &candidate : candidates) {
    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(candidate.c_str(), &error);
    if (error == nullptr) {
      if (pipeline != nullptr) {
        gst_object_unref(pipeline);
      }
      return true;
    }

    g_error_free(error);
    if (pipeline != nullptr) {
      gst_object_unref(pipeline);
    }
  }

  RCLCPP_ERROR(logger,
               "Unable to parse GStreamer stream for '%s'; expected either a "
               "full pipeline or a source snippet",
               name.c_str());
  return false;
}

void warn_if_interlaced_stream(const std::string &stream,
                               const rclcpp::Logger &logger,
                               const std::string &name) {
  if (stream.empty()) {
    return;
  }

  const std::string probe_pipeline =
      stream + " ! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 "
               "leaky=downstream"
               " ! appsink name=__caps_probe_sink__ sync=false async=false "
               "emit-signals=false drop=true max-buffers=1";

  GError *error = nullptr;
  GstElement *pipeline = gst_parse_launch(probe_pipeline.c_str(), &error);
  if (error != nullptr || pipeline == nullptr) {
    if (error != nullptr) {
      RCLCPP_WARN(logger, "Unable to probe caps for '%s' stream: %s",
                  name.c_str(),
                  error->message != nullptr ? error->message : "unknown error");
      g_error_free(error);
    }
    if (pipeline != nullptr) {
      gst_object_unref(pipeline);
    }
    return;
  }

  GstElement *probe_sink =
      gst_bin_get_by_name(GST_BIN(pipeline), "__caps_probe_sink__");
  if (probe_sink == nullptr) {
    gst_object_unref(pipeline);
    RCLCPP_WARN(logger,
                "Unable to probe caps for '%s' stream: missing probe sink",
                name.c_str());
    return;
  }

  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  GstSample *sample =
      gst_app_sink_try_pull_sample(GST_APP_SINK(probe_sink), 2 * GST_SECOND);

  if (sample != nullptr) {
    GstCaps *caps = gst_sample_get_caps(sample);
    if (caps != nullptr && gst_caps_get_size(caps) > 0) {
      const GstStructure *structure = gst_caps_get_structure(caps, 0);
      const gchar *interlace_mode =
          gst_structure_get_string(structure, "interlace-mode");
      if (interlace_mode != nullptr &&
          std::strcmp(interlace_mode, "progressive") != 0) {
        RCLCPP_WARN(logger,
                    "%s stream caps report interlace-mode='%s'. Consider "
                    "adding deinterlace to this stream in the config.",
                    name.c_str(), interlace_mode);
      }
    }
    gst_sample_unref(sample);
  }

  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(probe_sink);
  gst_object_unref(pipeline);
}

std::string resolve_unixfd_socket_path(const std::string &viewer_name,
                                      const sv::UnixfdSinkConfig &sink) {
  if (!sink.socket_path.empty()) {
    return sink.socket_path;
  }

  const char *username = getenv("USER");
  if (!username) {
    struct passwd *pw = getpwuid(getuid());
    username = pw ? pw->pw_name : "unknown";
  }

  std::string suffix = sink.name;
  if (suffix.empty()) {
    suffix = sink.stream;
  }

  return "/tmp/" + viewer_name + "_" + suffix + "_" + std::string(username) +
         ".sock";
}

bool check_element_available(const std::string &element_name) {
  GstElementFactory *factory = gst_element_factory_find(element_name.c_str());
  if (factory) {
    gst_object_unref(factory);
    return true;
  }
  return false;
}

std::string get_unixfd_upload_chain() {
  return "gldownload ! videoconvert ! video/x-raw,format=I420";
}

GstPadProbeReturn frame_timestamp_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                           gpointer user_data) {
  (void)pad;
  if (info->type & GST_PAD_PROBE_TYPE_BUFFER) {
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

    {
      std::scoped_lock<std::mutex> lock(state->mutex);
      if (element_name == "__left_src_q__") {
        timestamps.left_source_ts = now;
        state->left_source_ts = now;
      } else if (element_name == "__right_src_q__") {
        timestamps.right_source_ts = now;
        state->right_source_ts = now;
      } else {
        if (timestamps.left_source_ts == 0) {
          timestamps.left_source_ts = state->left_source_ts;
        }
        if (timestamps.right_source_ts == 0) {
          timestamps.right_source_ts = state->right_source_ts;
        }

        if (element_name.rfind("__stereo_unixfd_ts_q", 0) == 0 ||
            element_name == "__stereo_overlay_input_ts_q__") {
          timestamps.stereo_output_ts = now;
          state->stereo_output_ts = now;
        } else if (element_name.rfind("__overlay_unixfd_ts_q", 0) == 0) {
          if (timestamps.stereo_output_ts == 0) {
            timestamps.stereo_output_ts = state->stereo_output_ts;
          }
          timestamps.overlay_output_ts = now;
        }
      }
    }

    dc_buffer_set_frame_timestamps(buf, timestamps);
  }
  return GST_PAD_PROBE_OK;
}

void add_timestamp_probe(GstElement *pipeline, const std::string &element_name,
                         FrameTimestampState *timestamp_state) {
  GstElement *element =
      gst_bin_get_by_name(GST_BIN(pipeline), element_name.c_str());
  if (!element) {
    return;
  }
  GstPad *pad = gst_element_get_static_pad(element, "src");
  if (pad != nullptr) {
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
                      frame_timestamp_probe_cb, timestamp_state, nullptr);
    gst_object_unref(pad);
  }
  gst_object_unref(element);
}

void attach_timestamp_probes(GstElement *pipeline, const sv::AppConfig &cfg,
                             FrameTimestampState *timestamp_state) {
  add_timestamp_probe(pipeline, "__left_src_q__", timestamp_state);
  add_timestamp_probe(pipeline, "__right_src_q__", timestamp_state);
  add_timestamp_probe(pipeline, "__stereo_overlay_input_ts_q__", timestamp_state);

  int left_index = 0;
  int right_index = 0;
  int stereo_index = 0;
  int overlay_index = 0;
  for (const auto &sink : cfg.unixfd_sinks) {
    if (sink.stream == "left") {
      add_timestamp_probe(
          pipeline, "__left_unixfd_ts_q" + std::to_string(left_index++) + "__",
          timestamp_state);
    } else if (sink.stream == "right") {
      add_timestamp_probe(
          pipeline, "__right_unixfd_ts_q" + std::to_string(right_index++) + "__",
          timestamp_state);
    } else if (sink.stream == "stereo") {
      add_timestamp_probe(
          pipeline,
          "__stereo_unixfd_ts_q" + std::to_string(stereo_index++) + "__",
          timestamp_state);
    } else if (sink.stream == "overlay") {
      add_timestamp_probe(
          pipeline,
          "__overlay_unixfd_ts_q" + std::to_string(overlay_index++) + "__",
          timestamp_state);
    }
  }
}

gboolean on_sigint(gpointer) {
  if (g_app) {
    g_app->quit();
  }
  return G_SOURCE_REMOVE;
}

gboolean on_ros_spin(gpointer user_data) {
  if (user_data == nullptr || !rclcpp::ok()) {
    return G_SOURCE_CONTINUE;
  }

  auto *node = static_cast<rclcpp::Node *>(user_data);
  rclcpp::spin_some(node->get_node_base_interface());
  return G_SOURCE_CONTINUE;
}

gboolean on_bus_message(GstBus *, GstMessage *msg, gpointer user_data) {
  if (msg == nullptr) {
    return G_SOURCE_CONTINUE;
  }

  auto *node = static_cast<rclcpp::Node *>(user_data);

  if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
    RCLCPP_INFO(node->get_logger(), "GStreamer bus: received EOS");
    if (g_app) {
      g_app->quit();
    }
  } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
    GError *err = nullptr;
    gchar *dbg = nullptr;
    gst_message_parse_error(msg, &err, &dbg);
    RCLCPP_ERROR(node->get_logger(), "GStreamer error: %s", (err ? err->message : "unknown"));
    if (dbg != nullptr) {
      RCLCPP_ERROR(node->get_logger(), "Debug details: %s", dbg);
      g_free(dbg);
    }
    if (err != nullptr) {
      g_error_free(err);
    }
    if (g_app) {
      g_app->quit();
    }
  }

  return G_SOURCE_CONTINUE;
}

std::string color_adjustment_string(const sv::ColorAdjustment &color) {
  if (color.brightness == 0.0 && color.contrast == 1.0 &&
      color.saturation == 1.0 && color.hue == 0.0) {
    return "";
  }
  return " ! videobalance brightness=" + std::to_string(color.brightness) +
         " contrast=" + std::to_string(color.contrast) +
         " saturation=" + std::to_string(color.saturation) +
         " hue=" + std::to_string(color.hue);
}

// Helper: ensure a pixel count is even (round down)
static int make_even(int v) { return v & ~1; }

std::string
build_pipeline_string(const sv::AppConfig &stereo, const bool include_overlay) {
  int base_crop_w =
      stereo.crop_width > 0 ? stereo.crop_width : stereo.original_width;
  int base_crop_h =
      stereo.crop_height > 0 ? stereo.crop_height : stereo.original_height;
  base_crop_w =
      normalize_eye_size_for_even_crop(stereo.original_width, base_crop_w);
  base_crop_h =
      normalize_eye_size_for_even_crop(stereo.original_height, base_crop_h);

  int aspect_crop_l = 0, aspect_crop_r = 0, aspect_crop_t = 0,
      aspect_crop_b = 0;

  if (stereo.preserve_size && stereo.original_width > 0 &&
      stereo.original_height > 0) {
    double orig_aspect = static_cast<double>(stereo.original_width) /
                         static_cast<double>(stereo.original_height);
    int w_c_prime = std::min(
        base_crop_w, static_cast<int>(std::round(base_crop_h * orig_aspect)));
    int h_c_prime = std::min(
        base_crop_h, static_cast<int>(std::round(base_crop_w / orig_aspect)));

    int diff_x = base_crop_w - w_c_prime;
    int diff_y = base_crop_h - h_c_prime;

    // Round up to even so the resulting crop dimensions stay even (required
    // for I420/NV12 planar formats; odd widths/heights cause chroma artifacts).
    diff_x = (diff_x + 1) & ~1;
    diff_y = (diff_y + 1) & ~1;

    aspect_crop_l = diff_x / 2;
    aspect_crop_r = diff_x - aspect_crop_l;
    aspect_crop_t = diff_y / 2;
    aspect_crop_b = diff_y - aspect_crop_t;
  }

  const int eye_w = stereo.preserve_size ? stereo.original_width : base_crop_w;
  const int eye_h = stereo.preserve_size ? stereo.original_height : base_crop_h;

  const bool has_glimage = std::find(stereo.sinks.begin(), stereo.sinks.end(),
                                     "glimage") != stereo.sinks.end();
  const bool has_glimages = std::find(stereo.sinks.begin(), stereo.sinks.end(),
                                      "glimages") != stereo.sinks.end();

  int horizontal_shift_px = clamp_offset_to_valid(
      stereo.original_width, base_crop_w, stereo.horizontal_shift_px);
  int vertical_shift_px = clamp_offset_to_valid(
      stereo.original_height, base_crop_h, stereo.vertical_shift_px);

  CropValues left_crop = compute_eye_crop(
      stereo.original_width, stereo.original_height, base_crop_w, base_crop_h,
      horizontal_shift_px, vertical_shift_px, -1);

  CropValues right_crop = compute_eye_crop(
      stereo.original_width, stereo.original_height, base_crop_w, base_crop_h,
      horizontal_shift_px, vertical_shift_px, 1);

  if (stereo.preserve_size) {
    left_crop.left += aspect_crop_l;
    left_crop.right += aspect_crop_r;
    left_crop.top += aspect_crop_t;
    left_crop.bottom += aspect_crop_b;

    right_crop.left += aspect_crop_l;
    right_crop.right += aspect_crop_r;
    right_crop.top += aspect_crop_t;
    right_crop.bottom += aspect_crop_b;
  }

  // Fold display_horizontal_offset_px into each eye's crop window so
  // the mixer stays a plain side-by-side compositor with no xpos shifting.
  {
    const int disp = stereo.display_horizontal_offset_px;
    const int disp_half_lo = disp / 2;
    const int disp_half_hi = disp - disp_half_lo;

    const int left_shift = std::min(disp_half_lo, left_crop.left);
    left_crop.left  -= left_shift;
    left_crop.right += left_shift;

    const int right_shift = std::min(disp_half_hi, right_crop.right);
    right_crop.left  += right_shift;
    right_crop.right -= right_shift;
  }

  std::string scale_suffix = "";
  if (stereo.preserve_size && stereo.original_width > 0 &&
      stereo.original_height > 0) {
    scale_suffix = " ! videoscale ! video/x-raw,width=" +
                   std::to_string(stereo.original_width) +
                   ",height=" + std::to_string(stereo.original_height);
  }

  std::vector<sv::UnixfdSinkConfig> left_unixfd_sinks;
  for (const auto &sink : stereo.unixfd_sinks) {
    if (sink.stream == "left") {
      left_unixfd_sinks.push_back(sink);
    }
  }

  std::string left_chain =
      stereo.left.source +
      " ! queue name=__left_src_q__ max-size-buffers=8 leaky=downstream " +
      color_adjustment_string(stereo.left_color) +
      " ! videocrop left=" + std::to_string(left_crop.left) +
      " right=" + std::to_string(left_crop.right) +
      " top=" + std::to_string(left_crop.top) +
      " bottom=" + std::to_string(left_crop.bottom) + scale_suffix;

  const bool need_left_tee = !left_unixfd_sinks.empty();

  if (need_left_tee) {
    left_chain += " ! tee name=__left_out__"
                  " __left_out__. ! queue max-size-buffers=1 leaky=downstream "
                  "! glupload ! mix.sink_0";
    int left_unixfd_index = 0;
    for (const auto &sink : left_unixfd_sinks) {
      const std::string socket_path =
          resolve_unixfd_socket_path(stereo.name, sink);
      left_chain += " __left_out__. ! queue max-size-buffers=2 "
                    "max-size-time=0 max-size-bytes=0 leaky=downstream"
                    " ! videoconvert ! video/x-raw,format=I420"
                    " ! queue name=__left_unixfd_ts_q" +
                    std::to_string(left_unixfd_index++) +
                    "__ max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
                    "leaky=downstream ! unixfdsink socket-path=" +
                    socket_path + " sync=true async=false";
    }
  } else {
    left_chain += " ! glupload ! mix.sink_0";
  }

  std::vector<sv::UnixfdSinkConfig> right_unixfd_sinks;
  for (const auto &sink : stereo.unixfd_sinks) {
    if (sink.stream == "right") {
      right_unixfd_sinks.push_back(sink);
    }
  }

  std::string right_chain =
      stereo.right.source +
      " ! queue name=__right_src_q__ max-size-buffers=8 leaky=downstream " +
      color_adjustment_string(stereo.right_color) +
      " ! videocrop left=" + std::to_string(right_crop.left) +
      " right=" + std::to_string(right_crop.right) +
      " top=" + std::to_string(right_crop.top) +
      " bottom=" + std::to_string(right_crop.bottom) + scale_suffix;

  const bool need_right_tee = !right_unixfd_sinks.empty();

  if (need_right_tee) {
    right_chain += " ! tee name=__right_out__"
                   " __right_out__. ! queue max-size-buffers=1 "
                   "leaky=downstream ! glupload ! mix.sink_1";
    int right_unixfd_index = 0;
    for (const auto &sink : right_unixfd_sinks) {
      const std::string socket_path =
          resolve_unixfd_socket_path(stereo.name, sink);
      right_chain += " __right_out__. ! queue max-size-buffers=2 "
                     "max-size-time=0 max-size-bytes=0 leaky=downstream"
                     " ! videoconvert ! video/x-raw,format=I420"
                     " ! queue name=__right_unixfd_ts_q" +
                     std::to_string(right_unixfd_index++) +
                     "__ max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
                     "leaky=downstream ! unixfdsink socket-path=" +
                     socket_path + " sync=true async=false";
    }
  } else {
    right_chain += " ! glupload ! mix.sink_1";
  }


  // -----------------------------------------------------------------------
  // Extra mono streams layout
  // -----------------------------------------------------------------------
  const auto &es = stereo.extra_streams;
  const int n_mono = static_cast<int>(es.monos.size());
  const int n_stereo = static_cast<int>(es.stereos.size());
  const int n_extra_streams = n_mono + n_stereo;
  const bool has_extra = n_extra_streams > 0 && es.scale > 0.01;

  // stereo_h: height occupied by the stereo region in the final eye frame
  // extra_h:  remaining height for mono streams
  // gap_px:   black spacer between regions and between streams
  int stereo_h = eye_h;
  int extra_h  = 0;
  int gap_px   = 0;

  if (has_extra) {
    gap_px   = sv::AppConfig::gap_px;
    stereo_h = make_even(static_cast<int>(std::round(eye_h * (1.0 - es.scale))));
    stereo_h = std::max(2, std::min(eye_h - 2, stereo_h));
    extra_h  = eye_h - stereo_h; // includes the gap row
  }

  // slot_w: width of each extra stream within one eye's view
  int slot_w = 0;
  if (has_extra && n_extra_streams > 0) {
    slot_w = make_even((eye_w - (n_extra_streams - 1) * gap_px) / n_extra_streams);
    slot_w = std::max(2, slot_w);
  }
  // height of the visible mono content (extra_h minus top gap)
  const int mono_h = has_extra ? std::max(2, extra_h - gap_px) : 0;

  // Mono source chains: each stream is tee'd so it can feed both left and
  // right eye compositors (and the combined stereo compositor).
  // We do NOT glupload before the tee — keep in CPU memory so downstream
  // videoscale elements can work without gldownload/glupload round-trips.
  // -----------------------------------------------------------------------
  std::string extra_chains;
  if (has_extra) {
    for (int i = 0; i < n_mono; ++i) {
      extra_chains += es.monos[i] +
          " ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0"
          " leaky=downstream"
          " ! identity sync=true single-segment=true"
          " ! tee name=__extra_mono" + std::to_string(i) + "__  ";
    }
    for (int i = 0; i < n_stereo; ++i) {
      extra_chains += es.stereos[i].left +
          " ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0"
          " leaky=downstream"
          " ! identity sync=true single-segment=true"
          " ! tee name=__extra_stereo_left" + std::to_string(i) + "__  ";
      extra_chains += es.stereos[i].right +
          " ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0"
          " leaky=downstream"
          " ! identity sync=true single-segment=true"
          " ! tee name=__extra_stereo_right" + std::to_string(i) + "__  ";
    }
  }

  // Helper: append branches from extra tees into a named glvideomixer.
  // mix_name:  name of the destination glvideomixer
  // start_sink: first sink index (sink_0 is already used by stereo)
  // is_left_eye: true if this compositor is for the left eye
  auto append_extra_branches = [&](std::string &chain, const std::string &mix_name,
                                   int start_sink, bool is_left_eye) {
    for (int i = 0; i < n_mono; ++i) {
      const int sink_idx = start_sink + i;
      chain += " __extra_mono" + std::to_string(i) + "__. "
               "! queue max-size-buffers=2 leaky=downstream"
               " ! glupload ! " + mix_name + ".sink_" + std::to_string(sink_idx) + " ";
    }
    for (int i = 0; i < n_stereo; ++i) {
      const int sink_idx = start_sink + n_mono + i;
      std::string tee_name = is_left_eye ? "__extra_stereo_left" : "__extra_stereo_right";
      chain += " " + tee_name + std::to_string(i) + "__. "
               "! queue max-size-buffers=2 leaky=downstream"
               " ! glupload ! " + mix_name + ".sink_" + std::to_string(sink_idx) + " ";
    }
  };

  std::string output_chain =
      "glvideomixer name=mix background=1"
      " sink_0::xpos=0 sink_0::width=" + std::to_string(eye_w) +
      " sink_0::height=" + std::to_string(eye_h) +
      " sink_1::xpos=" + std::to_string(eye_w) +
      " sink_1::width=" + std::to_string(eye_w) +
      " sink_1::height=" + std::to_string(eye_h) +
      " ! video/x-raw(memory:GLMemory),width=" + std::to_string(2 * eye_w) +
      ",height=" + std::to_string(eye_h);

  std::vector<sv::UnixfdSinkConfig> stereo_unixfd_sinks;
  std::vector<sv::UnixfdSinkConfig> overlay_unixfd_sinks;
  for (const auto &sink : stereo.unixfd_sinks) {
    if (sink.stream == "stereo") {
      stereo_unixfd_sinks.push_back(sink);
    } else if (sink.stream == "overlay") {
      overlay_unixfd_sinks.push_back(sink);
    }
  }


  const bool has_display_output = has_glimage || has_glimages;

  int stereo_branches = (has_display_output ? 1 : 0) +
                        stereo_unixfd_sinks.size() +
                        (overlay_unixfd_sinks.empty() ? 0 : 1);


  if (stereo_branches > 0) {
    if (stereo_branches > 1) {
      output_chain += " ! tee name=__stereo_out__ ";
    }
    
    if (has_display_output) {
      if (stereo_branches > 1) {
        output_chain += " __stereo_out__. ! queue max-size-buffers=1 leaky=downstream";
      } else {
        // Preserve non-blocking behavior for the single display branch.
        output_chain += " ! queue max-size-buffers=1 leaky=downstream";
      }

      if (has_extra) {
        if (include_overlay) {
          output_chain +=
              " ! gldownload ! videoconvert ! cairooverlay name=stereo_overlay"
              " ! videoconvert ! video/x-raw,format=RGBA,width=" +
              std::to_string(2 * eye_w) + ",height=" + std::to_string(eye_h) +
              " ! tee name=__stereo_split__ ";
        } else {
          output_chain += " ! gldownload ! videoconvert ! video/x-raw,format=RGBA,width=" +
                          std::to_string(2 * eye_w) + ",height=" + std::to_string(eye_h) +
                          " ! tee name=__stereo_split__ ";
        }
        // Left half (crops out the right half)
        output_chain += " __stereo_split__. ! queue max-size-buffers=1 leaky=downstream"
                        " ! videocrop left=0 right=" + std::to_string(eye_w) + " top=0 bottom=0 "
                        " ! video/x-raw,format=RGBA,width=" + std::to_string(eye_w) + ",height=" + std::to_string(eye_h) +
                        " ! glupload ! __stereo_comp__.sink_0 ";
        // Right half (crops out the left half)
        output_chain += " __stereo_split__. ! queue max-size-buffers=1 leaky=downstream"
                        " ! videocrop left=" + std::to_string(eye_w) + " right=0 top=0 bottom=0 "
                        " ! video/x-raw,format=RGBA,width=" + std::to_string(eye_w) + ",height=" + std::to_string(eye_h) +
                        " ! glupload ! __stereo_comp__.sink_1 ";
        // Extra branches — left half uses sink_2..n_extra_streams+1
        // and right half uses sink_n_extra_streams+2..2*n_extra_streams+1
        append_extra_branches(output_chain, "__stereo_comp__", 2, true);
        append_extra_branches(output_chain, "__stereo_comp__", 2 + n_extra_streams, false);

        // Build the combined stereo compositor description
        std::string comp_desc =
            "glvideomixer name=__stereo_comp__ background=1"
            " sink_0::xpos=0 sink_0::ypos=0"
            " sink_0::width=" + std::to_string(eye_w) +
            " sink_0::height=" + std::to_string(stereo_h) +
            " sink_0::sizing-policy=1"
            " sink_1::xpos=" + std::to_string(eye_w) + " sink_1::ypos=0"
            " sink_1::width=" + std::to_string(eye_w) +
            " sink_1::height=" + std::to_string(stereo_h) +
            " sink_1::sizing-policy=1";
        const int disp = stereo.display_horizontal_offset_px;
        const int disp_half_lo = disp / 2;
        const int disp_half_hi = disp - disp_half_lo;
        for (int i = 0; i < n_extra_streams; ++i) {
          const int base_x = i * (slot_w + gap_px);
          const int left_x = std::max(0, std::min(eye_w - slot_w,
                                                   base_x - disp_half_lo));
          const int right_x = std::max(eye_w,
                                       std::min((2 * eye_w) - slot_w,
                                                eye_w + base_x + disp_half_hi));
          // Left half
          comp_desc +=
              " sink_" + std::to_string(2 + i) +
              "::xpos=" + std::to_string(left_x) +
              " sink_" + std::to_string(2 + i) +
              "::ypos=" + std::to_string(stereo_h + gap_px) +
              " sink_" + std::to_string(2 + i) +
              "::width=" + std::to_string(slot_w) +
              " sink_" + std::to_string(2 + i) +
              "::height=" + std::to_string(mono_h) +
              " sink_" + std::to_string(2 + i) + "::sizing-policy=1" +
              " sink_" + std::to_string(2 + i) + "::yalign=0.0";
          // Right half
          comp_desc +=
              " sink_" + std::to_string(2 + n_extra_streams + i) +
              "::xpos=" + std::to_string(right_x) +
              " sink_" + std::to_string(2 + n_extra_streams + i) +
              "::ypos=" + std::to_string(stereo_h + gap_px) +
              " sink_" + std::to_string(2 + n_extra_streams + i) +
              "::width=" + std::to_string(slot_w) +
              " sink_" + std::to_string(2 + n_extra_streams + i) +
              "::height=" + std::to_string(mono_h) +
              " sink_" + std::to_string(2 + n_extra_streams + i) + "::sizing-policy=1" +
              " sink_" + std::to_string(2 + n_extra_streams + i) + "::yalign=0.0";
        }
        output_chain += " " + comp_desc;
        output_chain +=
            " ! video/x-raw(memory:GLMemory),width=" +
            std::to_string(2 * eye_w) + ",height=" + std::to_string(eye_h) +
            " ! glcolorconvert";
      } else {
        // No extra streams
        if (include_overlay) {
          output_chain += " ! gldownload ! videoconvert ! cairooverlay "
                          "name=stereo_overlay ! glupload";
        }
        output_chain += " ! glcolorconvert";
      }

      const bool split_display = has_glimage || has_glimages;
      if (split_display) {
        output_chain += " ! tee name=__stereo_display_out__";
      }

      if (has_glimage) {
        output_chain +=
            " __stereo_display_out__. ! queue max-size-buffers=1 leaky=downstream"
            " ! gtkglsink sync=false force-aspect-ratio=false"
            " name=__stereo_sink__";
      }

      if (has_glimages) {
        output_chain +=
            " __stereo_display_out__. ! queue max-size-buffers=1 leaky=downstream"
        " ! gldownload ! videoconvert ! video/x-raw,format=RGBA,width=" +
        std::to_string(2 * eye_w) + ",height=" + std::to_string(eye_h) +
            " ! tee name=__stereo_eye_split__"
            " __stereo_eye_split__. ! queue max-size-buffers=1 leaky=downstream"
            " ! videocrop left=0 right=" + std::to_string(eye_w) +
            " top=0 bottom=0"
        " ! video/x-raw,format=RGBA,width=" + std::to_string(eye_w) +
        ",height=" + std::to_string(eye_h) +
            " ! glupload ! glcolorconvert ! gtkglsink name=__left_eye_sink__"
            " sync=false force-aspect-ratio=false"
            " __stereo_eye_split__. ! queue max-size-buffers=1 leaky=downstream"
            " ! videocrop left=" + std::to_string(eye_w) +
            " right=0 top=0 bottom=0"
        " ! video/x-raw,format=RGBA,width=" + std::to_string(eye_w) +
        ",height=" + std::to_string(eye_h) +
            " ! glupload ! glcolorconvert ! gtkglsink name=__right_eye_sink__"
            " sync=false force-aspect-ratio=false";
      }
    }

    int stereo_unixfd_index = 0;
    for (const auto &sink : stereo_unixfd_sinks) {
      const std::string socket_path =
          resolve_unixfd_socket_path(stereo.name, sink);
      const std::string unixfd_upload_chain = get_unixfd_upload_chain();
      if (stereo_branches > 1) {
        output_chain += " __stereo_out__. ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! ";
      } else {
        output_chain += " ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! ";
      }
      output_chain += unixfd_upload_chain +
                      " ! queue name=__stereo_unixfd_ts_q" +
                      std::to_string(stereo_unixfd_index++) +
                      "__ max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
                      "leaky=downstream ! unixfdsink socket-path=" +
                      socket_path + " sync=true async=false";
    }

    if (!overlay_unixfd_sinks.empty()) {
      if (stereo_branches > 1) {
        output_chain += " __stereo_out__. ! queue name=__stereo_overlay_input_ts_q__ max-size-buffers=1 leaky=downstream ! ";
      } else {
        output_chain += " ! queue name=__stereo_overlay_input_ts_q__ max-size-buffers=1 leaky=downstream ! ";
      }
      output_chain += "gldownload ! videoconvert ! cairooverlay name=stereo_overlay_unixfd ";
      
      if (overlay_unixfd_sinks.size() > 1) {
        output_chain += "! tee name=__overlay_out__ ";
      }
      
      int overlay_unixfd_index = 0;
      for (const auto &sink : overlay_unixfd_sinks) {
        const std::string socket_path =
            resolve_unixfd_socket_path(stereo.name, sink);
        if (overlay_unixfd_sinks.size() > 1) {
          output_chain += " __overlay_out__. ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! ";
        }
        output_chain += "videoconvert ! video/x-raw,format=I420"
                        " ! queue name=__overlay_unixfd_ts_q" +
                        std::to_string(overlay_unixfd_index++) +
                        "__ max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
                        "leaky=downstream ! unixfdsink socket-path=" +
                        socket_path + " sync=true async=false";
      }
    }
  } else {
    if (has_extra) {
      // No display sink but we still compose (rare: fakesink)
      if (include_overlay) {
        output_chain +=
            " ! gldownload ! videoconvert ! cairooverlay name=stereo_overlay";
      }
    } else {
      if (include_overlay) {
        output_chain +=
            " ! gldownload ! videoconvert ! cairooverlay name=stereo_overlay";
      }
    }
    output_chain += " ! fakesink sync=false";
  }

  return extra_chains + left_chain + " " + right_chain + " " + output_chain;
}

class ControlWindow : public Gtk::Window {
public:
  using RebuildCb = std::function<void(double new_scale)>;

  ControlWindow(std::shared_ptr<sv::OverlayState> overlay_state,
                GstElement *pipeline,
                const sv::AppConfig &cfg,
                RebuildCb rebuild_cb)
      : m_overlay_state(overlay_state), m_pipeline(pipeline),
        m_cfg(cfg), m_rebuild_cb(std::move(rebuild_cb)),
        m_display_outputs(cfg, "stereo", [this]() { on_quit_clicked(); }) {
    set_title("dVRK Display Control");
    set_border_width(10);
    m_vbox.set_orientation(Gtk::ORIENTATION_VERTICAL);
    m_vbox.set_spacing(8);
    add(m_vbox);

    m_btn_overlay.set_label("Overlay");
    m_btn_overlay.set_active(true);
    m_btn_overlay.signal_toggled().connect(
        sigc::mem_fun(*this, &ControlWindow::on_overlay_toggled));
    m_vbox.pack_start(m_btn_overlay, Gtk::PACK_SHRINK);
    m_vbox.pack_start(m_display_outputs.widget(), Gtk::PACK_SHRINK);

    if (!m_cfg.extra_streams.monos.empty() || !m_cfg.extra_streams.stereos.empty()) {
      m_scale_label.set_text("Extra Streams");
      m_scale_label.set_halign(Gtk::ALIGN_START);
      m_vbox.pack_start(m_scale_label, Gtk::PACK_SHRINK);
      
      const int n_mono = static_cast<int>(m_cfg.extra_streams.monos.size());
      const int n_stereo = static_cast<int>(m_cfg.extra_streams.stereos.size());
      const int n_extra = n_mono + n_stereo;
      const int gap = sv::AppConfig::gap_px;
      // Calculate max useful scale (assuming 4:3 source ratio bounding the width)
      int eye_w = m_cfg.original_width;
      if (eye_w == 0) eye_w = 640; // fallback
      int eye_h = m_cfg.original_height;
      if (eye_h == 0) eye_h = 480; // fallback
      
      int slot_w = (eye_w - (n_extra - 1) * gap) / n_extra;
      int max_mono_h = slot_w * 3 / 4; 
      double max_scale = static_cast<double>(max_mono_h + gap) / eye_h;
      max_scale = std::min(0.95, std::max(0.1, max_scale));

      m_scale_slider.set_range(0.00, max_scale);
      m_scale_slider.set_value(std::min(m_cfg.extra_streams.scale, max_scale));
      m_scale_slider.set_digits(2);
      m_scale_slider.set_draw_value(true);
      m_scale_slider.set_increments(0.01, 0.05);
      m_scale_slider.set_size_request(220, -1);
      m_scale_slider.signal_button_release_event().connect(
          [this](GdkEventButton *) -> bool {
            on_scale_released();
            return false;
          });
      m_vbox.pack_start(m_scale_slider, Gtk::PACK_SHRINK);
      m_scale_visible = true;
    }

    m_btn_quit.set_label("Quit");
    m_btn_quit.signal_clicked().connect(
        sigc::mem_fun(*this, &ControlWindow::on_quit_clicked));
    m_vbox.pack_start(m_btn_quit, Gtk::PACK_SHRINK);

    const int height = 180 + (m_scale_visible ? 60 : 0);
    set_default_size(260, height);
    add_events(Gdk::KEY_PRESS_MASK);
    show_all_children();
    setup_display_windows(pipeline);
  }

  void setup_display_windows(GstElement *pipeline) {
    m_pipeline = pipeline;
    std::vector<sv::DisplayOutputPanel::SinkDescriptor> sinks;
    const std::vector<std::string> sink_names = {
        "__left_eye_sink__", "__right_eye_sink__", "__stereo_sink__"};
    for (const auto &sname : sink_names) {
      GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), sname.c_str());
      if (!sink) continue;
      GtkWidget *gtk_widget = nullptr;
      g_object_get(sink, "widget", &gtk_widget, NULL);
      gst_object_unref(sink);
      if (!gtk_widget) continue;
      sv::DisplayOutputPanel::SinkDescriptor desc;
      desc.sink_name = sname;
      desc.label = sname == "__stereo_sink__"    ? "Stereo"
                   : sname == "__left_eye_sink__" ? "Left Eye"
                                                  : "Right Eye";
      desc.window_title = sname == "__stereo_sink__"    ? "Stereo Display"
                          : sname == "__left_eye_sink__" ? "Left Eye Display"
                                                         : "Right Eye Display";
      desc.default_width = 1280;
      desc.default_height = 720;
      desc.gtk_widget = gtk_widget;
      sinks.push_back(std::move(desc));
    }
    m_display_outputs.rebuild(sinks);
    show_all_children();
  }

protected:
  bool on_key_press_event(GdkEventKey *event) override {
    if ((event->state & GDK_CONTROL_MASK) && (event->keyval == GDK_KEY_q)) {
      on_quit_clicked();
      return true;
    }
    return Gtk::Window::on_key_press_event(event);
  }

  void on_overlay_toggled() {
    std::scoped_lock<std::mutex> lock(m_overlay_state->mutex);
    m_overlay_state->overlay_enabled = m_btn_overlay.get_active();
  }

  void on_scale_released() {
    if (m_rebuild_cb) m_rebuild_cb(m_scale_slider.get_value());
  }

  void on_quit_clicked() {
    if (g_app) g_app->quit();
  }

  std::shared_ptr<sv::OverlayState> m_overlay_state;
  GstElement *m_pipeline;
  const sv::AppConfig &m_cfg;
  RebuildCb m_rebuild_cb;
  bool m_scale_visible = false;
  Gtk::Box m_vbox;
  Gtk::ToggleButton m_btn_overlay;
  Gtk::Button m_btn_quit;
  Gtk::Label m_scale_label;
  Gtk::Scale m_scale_slider{Gtk::ORIENTATION_HORIZONTAL};
  sv::DisplayOutputPanel m_display_outputs;
};
} // namespace

int main(int argc, char *argv[]) {
  // gtkglsink requires OpenGL via GLX. Force the x11 GDK backend so that
  // XWayland is used on Wayland sessions. On pure X11 this is a no-op.
  // Users can override by setting GDK_BACKEND before launching the node.
  if (!getenv("GDK_BACKEND")) {
    setenv("GDK_BACKEND", "x11", 0);
  }

  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);

  CommandLineOptions options;
  if (!parse_arguments(argc, argv, options)) {
    print_usage(argv[0]);
    rclcpp::shutdown();
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("dvrk_display");
  auto overlay_state = std::make_shared<sv::OverlayState>();

  std::string console_name = "console";

  const bool overlay_available = check_element_available("cairooverlay");
  if (!overlay_available) {
    RCLCPP_WARN(node->get_logger(),
                "GStreamer element 'cairooverlay' is unavailable; dVRK status "
                "overlay is disabled");
  }

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

  if (!sv::Config::check_type(root, "dd::display_config@1.0.0", path)) {
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

  if (cfg.sinks.empty()) {
    RCLCPP_WARN(node->get_logger(), "Config '%s' has an empty sinks list",
                cfg.name.c_str());
  } else {
    std::string configured_sinks;
    for (const auto &sink : cfg.sinks) {
      if (sink != "glimage" && sink != "glimages") {
        RCLCPP_ERROR(
            node->get_logger(),
            "Unsupported sink '%s'. Allowed values are: glimage, glimages",
            sink.c_str());
        rclcpp::shutdown();
        return 1;
      }

      if (!configured_sinks.empty()) {
        configured_sinks += ", ";
      }
      configured_sinks += sink;
    }
    RCLCPP_INFO(node->get_logger(), "Configured sinks: [%s]",
                configured_sinks.c_str());
  }

  std::string pipeline_string;

  RCLCPP_INFO(node->get_logger(), "Loaded viewer config: %s", cfg.name.c_str());
  console_name = cfg.dvrk_console_namespace;
  overlay_state->overlay_alpha = cfg.overlay_alpha;
  overlay_state->show_grid = options.show_grid;
  {
    // Scale offset from crop-pixel space to original-pixel space.
    const double offset_scale =
        (cfg.preserve_size && cfg.crop_width > 0 && cfg.original_width > 0)
            ? static_cast<double>(cfg.original_width) /
                  static_cast<double>(cfg.crop_width)
            : 1.0;
    overlay_state->display_horizontal_offset_px = static_cast<int>(
        std::round(cfg.display_horizontal_offset_px * offset_scale));
  }
  const sv::AppConfig &app_cfg = cfg;

  if (app_cfg.left.source.empty() || app_cfg.right.source.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Config '%s' must define left and right",
                 cfg.name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  warn_if_interlaced_stream(app_cfg.left.source, node->get_logger(), "left");
  warn_if_interlaced_stream(app_cfg.right.source, node->get_logger(), "right");

  if (app_cfg.crop_width <= 0 || app_cfg.crop_height <= 0 ||
      app_cfg.original_width <= 0 || app_cfg.original_height <= 0) {
    RCLCPP_ERROR(node->get_logger(),
                 "Config '%s' must provide positive "
                 "original_width/original_height and crop_width/crop_height",
                 cfg.name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  if (app_cfg.crop_width % 2 != 0 || app_cfg.crop_height % 2 != 0) {
    RCLCPP_WARN(
        node->get_logger(),
        "Config '%s' has odd crop dimensions (%dx%d). "
        "Cropping will be rounded to even values to prevent display artifacts.",
        cfg.name.c_str(), app_cfg.crop_width, app_cfg.crop_height);
  }

  const std::string unixfd_upload_chain = get_unixfd_upload_chain();
  if (!app_cfg.unixfd_sinks.empty()) {
    for (const auto &sink : app_cfg.unixfd_sinks) {
      const std::string socket_path =
          resolve_unixfd_socket_path(app_cfg.name, sink);
      RCLCPP_INFO(node->get_logger(), "unixfd sink: stream=%s path=%s",
                  sink.stream.c_str(), socket_path.c_str());
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "No unixfd sinks configured");
  }

  if (app_cfg.sink_streams.empty()) {
    RCLCPP_WARN(node->get_logger(), "Resolved sink_streams list is empty");
  } else {
    for (std::size_t i = 0; i < app_cfg.sink_streams.size(); ++i) {
      RCLCPP_INFO(node->get_logger(), "sink_streams[%zu]: %s", i,
                  app_cfg.sink_streams[i].c_str());
    }
  }

  pipeline_string = build_pipeline_string(app_cfg, overlay_available);

  const std::string camera_topic = "/" + console_name + "/camera";
  const std::string clutch_topic = "/" + console_name + "/clutch";
  const std::string operator_present_topic =
      "/" + console_name + "/operator_present";
  const std::string teleop_selected_topic =
      "/" + console_name + "/teleop/selected";
  const std::string teleop_unselected_topic =
      "/" + console_name + "/teleop/unselected";
  RCLCPP_INFO(node->get_logger(),
              "Console topics: camera=%s clutch=%s operator_present=%s "
              "teleop_selected=%s "
              "teleop_unselected=%s",
              camera_topic.c_str(), clutch_topic.c_str(),
              operator_present_topic.c_str(), teleop_selected_topic.c_str(),
              teleop_unselected_topic.c_str());

  const auto latch_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const auto measured_cp_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const auto persistent_event_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const auto teleop_selected_qos =
      rclcpp::QoS(rclcpp::KeepLast(5)).reliable().transient_local();
  auto camera_sub = node->create_subscription<sensor_msgs::msg::Joy>(
      camera_topic, latch_qos,
      [overlay_state](const sensor_msgs::msg::Joy::SharedPtr msg) {
        sv::on_camera_joy(msg, overlay_state);
      });

  auto clutch_sub = node->create_subscription<sensor_msgs::msg::Joy>(
      clutch_topic, latch_qos,
      [overlay_state](const sensor_msgs::msg::Joy::SharedPtr msg) {
        sv::on_clutch_joy(msg, overlay_state);
      });

  auto operator_present_sub = node->create_subscription<sensor_msgs::msg::Joy>(
      operator_present_topic, latch_qos,
      [overlay_state](const sensor_msgs::msg::Joy::SharedPtr msg) {
        sv::on_operator_present(msg, overlay_state);
      });
 
  auto ecm_js_sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "/ECM/measured_js", measured_cp_qos,
      [overlay_state](const sensor_msgs::msg::JointState::SharedPtr msg) {
        sv::on_ecm_measured_js(msg, overlay_state);
      });

  auto following_subscribers_cache = std::make_shared<std::unordered_map<
      std::string, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>>();
  auto measured_cp_subscribers_cache = std::make_shared<std::unordered_map<
      std::string,
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>>();
  auto tool_type_subscribers_cache = std::make_shared<std::unordered_map<
      std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr>>();
  auto scale_subscribers_cache = std::make_shared<std::unordered_map<
      std::string, rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr>>();
  auto state_subscribers_cache = std::make_shared<std::unordered_map<
      std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr>>();
  auto active_teleops = std::make_shared<std::unordered_set<std::string>>();
  auto latest_teleop_by_mtm =
      std::make_shared<std::unordered_map<std::string, std::string>>();
  auto teleop_selected_sub = node->create_subscription<std_msgs::msg::String>(
      teleop_selected_topic, teleop_selected_qos,
      [node, overlay_state, following_subscribers_cache,
       measured_cp_subscribers_cache, tool_type_subscribers_cache,
       scale_subscribers_cache, state_subscribers_cache, active_teleops,
       latest_teleop_by_mtm, latch_qos, measured_cp_qos,
       persistent_event_qos](const std_msgs::msg::String::SharedPtr msg) {
        if (msg == nullptr) {
          return;
        }

        std::string mtm_name;
        sv::TeleopSide side;
        int psm_number = 0;
        std::string arm_name;
        bool is_camera_teleop = false;
        if (!sv::parse_teleop_name(msg->data, mtm_name, side, psm_number,
                                   &arm_name, &is_camera_teleop)) {
          return;
        }

        const std::string teleop_name = msg->data;
        const auto latest_it = latest_teleop_by_mtm->find(mtm_name);
        if (latest_it != latest_teleop_by_mtm->end() &&
            latest_it->second != teleop_name) {
          const std::string previous_teleop = latest_it->second;
          active_teleops->erase(previous_teleop);
          auto previous_msg = std::make_shared<std_msgs::msg::String>();
          previous_msg->data = previous_teleop;
          sv::on_teleop_unselected(previous_msg, overlay_state);
          RCLCPP_INFO(node->get_logger(), "Replacing teleop for %s: %s -> %s",
                      mtm_name.c_str(), previous_teleop.c_str(),
                      teleop_name.c_str());
        }
        (*latest_teleop_by_mtm)[mtm_name] = teleop_name;
        active_teleops->insert(teleop_name);

        sv::on_teleop_selected(msg, overlay_state);

        if (following_subscribers_cache->find(teleop_name) ==
            following_subscribers_cache->end()) {
          const std::string following_topic = "/" + teleop_name + "/following";

          auto following_sub = node->create_subscription<std_msgs::msg::Bool>(
              following_topic, latch_qos,
              [overlay_state, active_teleops, teleop_name](
                  const std_msgs::msg::Bool::SharedPtr following_msg) {
                if (active_teleops->find(teleop_name) ==
                    active_teleops->end()) {
                  return;
                }
                sv::on_teleop_following(teleop_name, following_msg,
                                        overlay_state);
              });

          (*following_subscribers_cache)[teleop_name] = following_sub;
          RCLCPP_INFO(node->get_logger(),
                      "Cached teleop following subscriber: %s",
                      following_topic.c_str());
        }

        if (scale_subscribers_cache->find(teleop_name) ==
            scale_subscribers_cache->end()) {
          const std::string scale_topic = "/" + teleop_name + "/scale";

          auto scale_sub = node->create_subscription<std_msgs::msg::Float64>(
              scale_topic, latch_qos,
              [overlay_state, active_teleops,
               teleop_name](const std_msgs::msg::Float64::SharedPtr scale_msg) {
                if (active_teleops->find(teleop_name) ==
                    active_teleops->end()) {
                  return;
                }
                sv::on_teleop_scale(teleop_name, scale_msg, overlay_state);
              });

          (*scale_subscribers_cache)[teleop_name] = scale_sub;
          RCLCPP_INFO(node->get_logger(), "Cached teleop scale subscriber: %s",
                      scale_topic.c_str());
        }

        if (state_subscribers_cache->find(teleop_name) ==
            state_subscribers_cache->end()) {
          const std::string state_topic = "/" + teleop_name + "/current_state";

          auto state_sub = node->create_subscription<std_msgs::msg::String>(
              state_topic, latch_qos,
              [overlay_state, active_teleops,
               teleop_name](const std_msgs::msg::String::SharedPtr state_msg) {
                if (active_teleops->find(teleop_name) ==
                    active_teleops->end()) {
                  return;
                }
                sv::on_teleop_current_state(teleop_name, state_msg,
                                            overlay_state);
              });

          (*state_subscribers_cache)[teleop_name] = state_sub;
          RCLCPP_INFO(node->get_logger(), "Cached teleop state subscriber: %s",
                      state_topic.c_str());
        }

        if (!arm_name.empty() &&
            measured_cp_subscribers_cache->find(arm_name) ==
                measured_cp_subscribers_cache->end()) {
          const std::string measured_cp_topic = "/" + arm_name + "/measured_cp";
          auto measured_cp_sub =
              node->create_subscription<geometry_msgs::msg::PoseStamped>(
                  measured_cp_topic, measured_cp_qos,
                  [overlay_state,
                   arm_name](const geometry_msgs::msg::PoseStamped::SharedPtr
                                 measured_cp_msg) {
                    sv::on_teleop_measured_cp(arm_name, measured_cp_msg,
                                              overlay_state);
                  });

          (*measured_cp_subscribers_cache)[arm_name] = measured_cp_sub;
          RCLCPP_INFO(node->get_logger(),
                      "Cached arm measured_cp subscriber: %s",
                      measured_cp_topic.c_str());
        }

        if (!is_camera_teleop && psm_number > 0) {
          const std::string psm_name = "PSM" + std::to_string(psm_number);
          if (tool_type_subscribers_cache->find(psm_name) ==
              tool_type_subscribers_cache->end()) {
            const std::string tool_type_topic = "/" + psm_name + "/tool_type";
            auto tool_type_sub =
                node->create_subscription<std_msgs::msg::String>(
                    tool_type_topic, persistent_event_qos,
                    [overlay_state, psm_name](
                        const std_msgs::msg::String::SharedPtr tool_type_msg) {
                      sv::on_teleop_tool_type(psm_name, tool_type_msg,
                                              overlay_state);
                    });

            (*tool_type_subscribers_cache)[psm_name] = tool_type_sub;
            RCLCPP_INFO(node->get_logger(),
                        "Cached PSM tool_type subscriber: %s",
                        tool_type_topic.c_str());
          }
        }
      });

  auto teleop_unselected_sub = node->create_subscription<std_msgs::msg::String>(
      teleop_unselected_topic, latch_qos,
      [node, overlay_state, active_teleops,
       latest_teleop_by_mtm](const std_msgs::msg::String::SharedPtr msg) {
        if (msg == nullptr) {
          return;
        }

        std::string mtm_name;
        sv::TeleopSide side;
        int psm_number = 0;
        const bool parsed =
            sv::parse_teleop_name(msg->data, mtm_name, side, psm_number);

        sv::on_teleop_unselected(msg, overlay_state);
        active_teleops->erase(msg->data);
        RCLCPP_INFO(node->get_logger(),
                    "Teleop inactive (subscriber cached): /%s/following",
                    msg->data.c_str());

        if (parsed) {
          const auto latest_it = latest_teleop_by_mtm->find(mtm_name);
          if (latest_it != latest_teleop_by_mtm->end() &&
              latest_it->second == msg->data) {
            latest_teleop_by_mtm->erase(latest_it);
          }
        }
      });

  if (!validate_pipeline(pipeline_string, node->get_logger(),
                         "dvrk_display_pipeline")) {
    rclcpp::shutdown();
    return 1;
  }

  GError *error = nullptr;
  GstElement *pipeline = gst_parse_launch(pipeline_string.c_str(), &error);
  if (error != nullptr || pipeline == nullptr) {
    if (error != nullptr) {
      RCLCPP_ERROR(node->get_logger(), "Failed to create pipeline: %s",
                   error->message);
      g_error_free(error);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to create pipeline");
    }
    if (pipeline != nullptr) {
      gst_object_unref(pipeline);
    }
    rclcpp::shutdown();
    return 1;
  }

  GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  gst_bus_add_watch(bus, on_bus_message, node.get());
  gst_object_unref(bus);

  FrameTimestampState timestamp_state;
  attach_timestamp_probes(pipeline, cfg, &timestamp_state);

  // Helper: attach cairo overlay signals to all overlay elements in a pipeline.
  // Used for both initial setup and after pipeline rebuild.
  bool first_overlay_found = false;
  auto attach_overlays = [&](GstElement *pl) {
    if (!overlay_available) return;
    const std::vector<std::string> overlay_names = {
        "stereo_overlay", "stereo_overlay_unixfd", "left_overlay", "right_overlay"};
    for (const auto &overlay_name : overlay_names) {
      GstElement *overlay =
          gst_bin_get_by_name(GST_BIN(pl), overlay_name.c_str());
      if (overlay == nullptr) continue;
      first_overlay_found = true;
      g_signal_connect(overlay, "caps-changed",
                       G_CALLBACK(sv::on_overlay_caps_changed),
                       overlay_state.get());
      g_signal_connect(overlay, "draw", G_CALLBACK(sv::on_overlay_draw),
                       overlay_state.get());
      gst_object_unref(overlay);
    }
  };

  attach_overlays(pipeline);
  if (overlay_available && !first_overlay_found) {
    RCLCPP_WARN(node->get_logger(),
                "Unable to find overlay element in pipeline; dVRK status "
                "overlay is disabled");
  }


  g_app = Gtk::Application::create("org.dvrk.display.stereo." + app_cfg.name, Gio::APPLICATION_NON_UNIQUE);
  g_unix_signal_add(SIGINT, on_sigint, nullptr);
  g_unix_signal_add(SIGTERM, on_sigint, nullptr);
  g_timeout_add(20, on_ros_spin, node.get());

  rclcpp::spin_some(node->get_node_base_interface());

  (void)camera_sub;
  (void)clutch_sub;
  (void)operator_present_sub;
  (void)teleop_selected_sub;
  (void)teleop_unselected_sub;
  (void)ecm_js_sub;

  // Build the rebuild callback: stops old pipeline, rebuilds with new scale,
  // restarts, and refreshes the display windows in the control window.
  // control_window_ptr is set in signal_activate before any slider interaction.
  ControlWindow *control_window_ptr = nullptr;

  auto rebuild_pipeline = [&](double new_scale) {
    if (!control_window_ptr) return;

    RCLCPP_INFO(node->get_logger(),
                "Rebuilding pipeline with extra_streams.scale=%.2f", new_scale);

    // Stop and destroy old pipeline
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    pipeline = nullptr;

    // Update scale in config copy (cfg is a local copy in main)
    cfg.extra_streams.scale = new_scale;

    // Rebuild pipeline string
    const std::string new_pipeline_str =
        build_pipeline_string(cfg, overlay_available);

    GError *rebuild_error = nullptr;
    pipeline = gst_parse_launch(new_pipeline_str.c_str(), &rebuild_error);
    if (rebuild_error != nullptr || pipeline == nullptr) {
      RCLCPP_ERROR(node->get_logger(),
                   "Failed to rebuild pipeline after scale change: %s",
                   rebuild_error ? rebuild_error->message : "unknown");
      if (rebuild_error) g_error_free(rebuild_error);
      if (pipeline) { gst_object_unref(pipeline); pipeline = nullptr; }
      return;
    }

    // Re-attach bus watcher
    GstBus *new_bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    gst_bus_add_watch(new_bus, on_bus_message, node.get());
    gst_object_unref(new_bus);

    // Re-attach overlay signals
    attach_overlays(pipeline);
    attach_timestamp_probes(pipeline, cfg, &timestamp_state);

    // Refresh display windows in the control window
    control_window_ptr->setup_display_windows(pipeline);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    RCLCPP_INFO(node->get_logger(), "Pipeline restarted with new scale");
  };

  ControlWindow window(overlay_state, pipeline, cfg,
                       [&rebuild_pipeline](double s) { rebuild_pipeline(s); });
  control_window_ptr = &window;

  for (const auto &sink : app_cfg.unixfd_sinks) {
    const std::string socket_path =
        resolve_unixfd_socket_path(app_cfg.name, sink);
    if (std::filesystem::exists(socket_path)) {
      RCLCPP_INFO(node->get_logger(),
                  "Removing stale unixfd socket before starting pipeline: %s",
                  socket_path.c_str());
      std::error_code remove_error;
      if (!std::filesystem::remove(socket_path, remove_error) && remove_error) {
        RCLCPP_WARN(node->get_logger(),
                    "Unable to remove stale unixfd socket '%s': %s",
                    socket_path.c_str(), remove_error.message().c_str());
      }
    }
  }

  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  RCLCPP_INFO(node->get_logger(), "Stereo display pipeline started");

  if (options.dump_dot) {
    dc::dump_dot(pipeline, "stereo_pipeline.dot", options.dot_flags);
  }

  window.show();
  g_app->run(window);

  RCLCPP_INFO(node->get_logger(), "Stereo display pipeline on quit: %s",
              pipeline_string.c_str());

  if (pipeline) {
    gst_element_send_event(pipeline, gst_event_new_eos());
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
  }

  g_app.reset();

  rclcpp::shutdown();
  return 0;
}

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

#include <chrono>
#include <deque>
#include <iomanip>
#include <sstream>

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

#include <dvrk_data/config.hpp>
#include "display_output_panel.hpp"
#include <dvrk_data/gst_utils.hpp>
#include "overlay.hpp"
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <dvrk_data/stereo_common.hpp>

namespace {

class FpsTracker {
public:
  void update() {
    std::scoped_lock<std::mutex> lock(m_mutex);
    m_frame_times.push_back(std::chrono::steady_clock::now());
    m_total_count++;
  }

  double get_fps() {
    std::scoped_lock<std::mutex> lock(m_mutex);
    auto now = std::chrono::steady_clock::now();
    while (!m_frame_times.empty() && now - m_frame_times.front() > std::chrono::seconds(2)) {
      m_frame_times.pop_front();
    }
    if (m_frame_times.size() < 2) {
      return 0.0;
    }
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(m_frame_times.back() - m_frame_times.front()).count();
    if (duration == 0) return 0.0;
    return static_cast<double>(m_frame_times.size() - 1) / (duration / 1000.0);
  }

  uint64_t get_count() const {
    return m_total_count;
  }

private:
  std::mutex m_mutex;
  std::deque<std::chrono::steady_clock::time_point> m_frame_times;
  uint64_t m_total_count = 0;
};

static GstPadProbeReturn sink_fps_probe_cb(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
  (void)pad;
  if (info->type & GST_PAD_PROBE_TYPE_BUFFER) {
    auto *tracker = static_cast<FpsTracker *>(user_data);
    if (tracker) {
      tracker->update();
    }
  }
  return GST_PAD_PROBE_OK;
}

struct CommandLineOptions {
  std::string config_file;
  bool show_grid = false;
  bool dump_dot = false;
  GstDebugGraphDetails dot_flags = GST_DEBUG_GRAPH_SHOW_ALL;
};

static Glib::RefPtr<Gtk::Application> g_app;

struct FrameTimestampState {
  std::mutex mutex;
  gint64 left_source_ts = 0;
  gint64 right_source_ts = 0;
  gint64 stereo_output_ts = 0;
};

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
    if (arg == "--ros-args") {
      break;
    }

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

std::string resolve_unixfd_socket_path(const std::string &app_name,
                                       const sv::UnixfdSinkConfig &sink) {
  if (!sink.socket_path.empty()) {
    return sink.socket_path;
  }
  const char *username = getenv("USER");
  if (!username) {
    struct passwd *pw = getpwuid(getuid());
    username = pw ? pw->pw_name : "unknown";
  }
  const std::string suffix = sink.name.empty() ? sink.stream : sink.name;
  return "/tmp/" + app_name + "_" + suffix + "_" + std::string(username) + ".sock";
}


void warn_missing_unixfd_source_sockets(const sv::AppConfig &cfg,
                                        const rclcpp::Logger &logger) {
  for (const auto &src : cfg.unixfd_sources) {
    const char *username = getenv("USER");
    if (!username) {
      struct passwd *pw = getpwuid(getuid());
      username = pw ? pw->pw_name : "unknown";
    }
    const std::string path = src.socket_path.empty()
        ? "/tmp/" + cfg.name + "_" + src.name + "_" + std::string(username) + ".sock"
        : src.socket_path;
    if (!std::filesystem::exists(path)) {
      RCLCPP_WARN(
          logger,
          "unixfd source socket is missing before pipeline start: %s. "
          "Start the producer for this socket first; otherwise the display "
          "sinks can remain idle.",
          path.c_str());
    }
  }
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
      } else if (element_name == "__stereo_src_q__") {
        if (timestamps.left_source_ts != 0) {
          state->left_source_ts = timestamps.left_source_ts;
        }
        if (timestamps.right_source_ts != 0) {
          state->right_source_ts = timestamps.right_source_ts;
        }
        if (timestamps.stereo_output_ts != 0) {
          state->stereo_output_ts = timestamps.stereo_output_ts;
        }
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

GstPadProbeReturn ar_timestamp_probe_cb(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
  (void)user_data;
  if (info->type & GST_PAD_PROBE_TYPE_BUFFER) {
    GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!gst_buffer_is_writable(buf)) {
      buf = gst_buffer_make_writable(buf);
      GST_PAD_PROBE_INFO_DATA(info) = buf;
    }
    GstElement *element = gst_pad_get_parent_element(pad);
    if (element) {
      GstClockTime running_time = gst_element_get_current_running_time(element);
      if (GST_CLOCK_TIME_IS_VALID(running_time)) {
        GST_BUFFER_PTS(buf) = running_time;
        GST_BUFFER_DTS(buf) = GST_CLOCK_TIME_NONE;
      }
      gst_object_unref(element);
    }
  }
  return GST_PAD_PROBE_OK;
}

void attach_ar_timestamp_probes(GstElement *pipeline) {
  const std::vector<std::string> names = {"left_ar_src", "right_ar_src"};
  for (const auto &name : names) {
    GstElement *element = gst_bin_get_by_name(GST_BIN(pipeline), name.c_str());
    if (element) {
      GstPad *pad = gst_element_get_static_pad(element, "src");
      if (pad) {
        gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
                          ar_timestamp_probe_cb, nullptr, nullptr);
        gst_object_unref(pad);
      }
      gst_object_unref(element);
    }
  }
}

void attach_timestamp_probes(GstElement *pipeline, const sv::AppConfig &cfg,
                             FrameTimestampState *timestamp_state) {
  add_timestamp_probe(pipeline, "__stereo_src_q__", timestamp_state);
  add_timestamp_probe(pipeline, "__stereo_overlay_input_ts_q__", timestamp_state);

  int stereo_index = 0;
  int overlay_index = 0;
  for (const auto &sink : cfg.unixfd_sinks) {
    if (sink.stream == "stereo") {
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

// Removed local on_bus_message in favor of dc_stereo::on_bus_message

// Helper: ensure a pixel count is even (round down)
static int make_even(int v) { return v & ~1; }

std::string
build_pipeline_string(const sv::AppConfig &stereo, const bool include_overlay) {
  const int eye_w = stereo.original_width;
  const int eye_h = stereo.original_height;
  const int stereo_w = 2 * eye_w;

  const bool has_glimage = std::find(stereo.sinks.begin(), stereo.sinks.end(),
                                     "glimage") != stereo.sinks.end();
  const bool has_glimages = std::find(stereo.sinks.begin(), stereo.sinks.end(),
                                      "glimages") != stereo.sinks.end();

  const auto &es = stereo.extra_streams;
  const int n_mono = static_cast<int>(es.monos.size());
  const int n_stereo = static_cast<int>(es.stereos.size());
  const int n_extra_streams = n_mono + n_stereo;
  const bool has_extra = n_extra_streams > 0 && es.scale > 0.01;
  const bool has_ar = stereo.ar.enabled &&
                      (!stereo.ar.left_socket.empty() ||
                       !stereo.ar.right_socket.empty());
  const bool split_stereo_for_presentation = has_extra || has_ar;

  int stereo_h = eye_h;
  int extra_h = 0;
  int gap_px = 0;
  if (has_extra) {
    gap_px = sv::AppConfig::gap_px;
    stereo_h =
        make_even(static_cast<int>(std::round(eye_h * (1.0 - es.scale))));
    stereo_h = std::max(2, std::min(eye_h - 2, stereo_h));
    extra_h = eye_h - stereo_h;
  }

  int slot_w = 0;
  if (has_extra && n_extra_streams > 0) {
    slot_w =
        make_even((eye_w - (n_extra_streams - 1) * gap_px) / n_extra_streams);
    slot_w = std::max(2, slot_w);
  }
  const int mono_h = has_extra ? std::max(2, extra_h - gap_px) : 0;

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

  auto append_extra_branches = [&](std::string &chain,
                                   const std::string &mix_name,
                                   int start_sink, bool is_left_eye) {
    for (int i = 0; i < n_mono; ++i) {
      const int sink_idx = start_sink + i;
      chain += " __extra_mono" + std::to_string(i) + "__. "
               "! queue max-size-buffers=2 leaky=downstream"
               " ! glupload ! " + mix_name + ".sink_" +
               std::to_string(sink_idx) + " ";
    }
    for (int i = 0; i < n_stereo; ++i) {
      const int sink_idx = start_sink + n_mono + i;
      const std::string tee_name =
          is_left_eye ? "__extra_stereo_left" : "__extra_stereo_right";
      chain += " " + tee_name + std::to_string(i) + "__. "
               "! queue max-size-buffers=2 leaky=downstream"
               " ! glupload ! " + mix_name + ".sink_" +
               std::to_string(sink_idx) + " ";
    }
  };

  std::string input_chain;
  std::string presentation_chain;
  const std::string normalized_stereo_caps =
      " ! videoconvert ! video/x-raw,width=" + std::to_string(stereo_w) +
      ",height=" + std::to_string(eye_h);

  if (split_stereo_for_presentation) {
    input_chain =
        stereo.stereo.source +
        " ! queue name=__stereo_src_q__ max-size-buffers=8 "
        "max-size-time=0 max-size-bytes=0 leaky=downstream" +
        normalized_stereo_caps + " ! tee name=__clean_stereo__ ";

    std::string left_chain =
        " __clean_stereo__. ! queue max-size-buffers=2 leaky=downstream"
        " ! videocrop left=0 right=" + std::to_string(eye_w) +
        " top=0 bottom=0 ! video/x-raw,width=" + std::to_string(eye_w) +
        ",height=" + std::to_string(eye_h) + " ! glupload";

    if (stereo.ar.enabled && !stereo.ar.left_socket.empty()) {
      left_chain += " ! left_ar_mix.sink_0 ";
      left_chain +=
          "glvideomixer name=left_ar_mix background=1 force-live=true"
          " sink_0::zorder=1 sink_0::xpos=0 sink_0::ypos=0"
          " sink_0::width=" +
          std::to_string(eye_w) + " sink_0::height=" +
          std::to_string(eye_h) +
          " sink_1::zorder=2 sink_1::xpos=0 sink_1::ypos=0"
          " sink_1::width=" +
          std::to_string(eye_w) + " sink_1::height=" +
          std::to_string(eye_h) + " ";
      std::string alpha_str;
      if (stereo.ar.use_color_key) {
        alpha_str = " ! alpha method=custom target-r=" +
                    std::to_string(stereo.ar.color_key_r) +
                    " target-g=" + std::to_string(stereo.ar.color_key_g) +
                    " target-b=" + std::to_string(stereo.ar.color_key_b);
      }
      left_chain +=
          "unixfdsrc name=left_ar_src socket-path=" + stereo.ar.left_socket +
          " socket-type=abstract do-timestamp=true"
          " ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0"
          " leaky=downstream"
          " ! videoconvert ! video/x-raw,format=RGBA" +
          alpha_str + " ! glupload ! left_ar_mix.sink_1 ";
      left_chain +=
          "left_ar_mix. ! video/x-raw(memory:GLMemory),width=" +
          std::to_string(eye_w) + ",height=" + std::to_string(eye_h) +
          " ! mix.sink_0";
    } else {
      left_chain += " ! mix.sink_0";
    }

    std::string right_chain =
        " __clean_stereo__. ! queue max-size-buffers=2 leaky=downstream"
        " ! videocrop left=" + std::to_string(eye_w) +
        " right=0 top=0 bottom=0 ! video/x-raw,width=" +
        std::to_string(eye_w) + ",height=" + std::to_string(eye_h) +
        " ! glupload";

    if (stereo.ar.enabled && !stereo.ar.right_socket.empty()) {
      right_chain += " ! right_ar_mix.sink_0 ";
      right_chain +=
          "glvideomixer name=right_ar_mix background=1 force-live=true"
          " sink_0::zorder=1 sink_0::xpos=0 sink_0::ypos=0"
          " sink_0::width=" +
          std::to_string(eye_w) + " sink_0::height=" +
          std::to_string(eye_h) +
          " sink_1::zorder=2 sink_1::xpos=0 sink_1::ypos=0"
          " sink_1::width=" +
          std::to_string(eye_w) + " sink_1::height=" +
          std::to_string(eye_h) + " ";
      std::string alpha_str;
      if (stereo.ar.use_color_key) {
        alpha_str = " ! alpha method=custom target-r=" +
                    std::to_string(stereo.ar.color_key_r) +
                    " target-g=" + std::to_string(stereo.ar.color_key_g) +
                    " target-b=" + std::to_string(stereo.ar.color_key_b);
      }
      right_chain +=
          "unixfdsrc name=right_ar_src socket-path=" + stereo.ar.right_socket +
          " socket-type=abstract do-timestamp=true"
          " ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0"
          " leaky=downstream"
          " ! videoconvert ! video/x-raw,format=RGBA" +
          alpha_str + " ! glupload ! right_ar_mix.sink_1 ";
      right_chain +=
          "right_ar_mix. ! video/x-raw(memory:GLMemory),width=" +
          std::to_string(eye_w) + ",height=" + std::to_string(eye_h) +
          " ! mix.sink_1";
    } else {
      right_chain += " ! mix.sink_1";
    }

    input_chain += left_chain + right_chain;
    presentation_chain =
        " glvideomixer name=mix background=1"
        " sink_0::xpos=0 sink_0::width=" +
        std::to_string(eye_w) + " sink_0::height=" + std::to_string(eye_h) +
        " sink_1::xpos=" + std::to_string(eye_w) +
        " sink_1::width=" + std::to_string(eye_w) +
        " sink_1::height=" + std::to_string(eye_h) +
        " ! video/x-raw(memory:GLMemory),width=" + std::to_string(stereo_w) +
        ",height=" + std::to_string(eye_h);
  } else {
    presentation_chain =
        stereo.stereo.source +
        " ! queue name=__stereo_src_q__ max-size-buffers=8 "
        "max-size-time=0 max-size-bytes=0 leaky=downstream" +
        normalized_stereo_caps + " ! glupload ! glcolorconvert";
  }

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
  const int stereo_branches = (has_display_output ? 1 : 0) +
                              stereo_unixfd_sinks.size() +
                              (overlay_unixfd_sinks.empty() ? 0 : 1);

  std::string output_chain = presentation_chain;
  if (stereo_branches > 0) {
    if (stereo_branches > 1) {
      output_chain += " ! tee name=__stereo_out__ ";
    }

    if (has_display_output) {
      if (stereo_branches > 1) {
        output_chain +=
            " __stereo_out__. ! queue max-size-buffers=1 leaky=downstream";
      } else {
        output_chain += " ! queue max-size-buffers=1 leaky=downstream";
      }

      if (has_extra) {
        if (include_overlay) {
          output_chain +=
              " ! gldownload ! videoconvert ! cairooverlay name=stereo_overlay"
              " ! videoconvert ! video/x-raw,format=RGBA,width=" +
              std::to_string(stereo_w) + ",height=" + std::to_string(eye_h) +
              " ! tee name=__stereo_split__ ";
        } else {
          output_chain +=
              " ! gldownload ! videoconvert ! video/x-raw,format=RGBA,width=" +
              std::to_string(stereo_w) + ",height=" + std::to_string(eye_h) +
              " ! tee name=__stereo_split__ ";
        }
        output_chain +=
            " __stereo_split__. ! queue max-size-buffers=1 leaky=downstream"
            " ! videocrop left=0 right=" +
            std::to_string(eye_w) +
            " top=0 bottom=0 ! video/x-raw,format=RGBA,width=" +
            std::to_string(eye_w) + ",height=" + std::to_string(eye_h) +
            " ! glupload ! __stereo_comp__.sink_0 ";
        output_chain +=
            " __stereo_split__. ! queue max-size-buffers=1 leaky=downstream"
            " ! videocrop left=" +
            std::to_string(eye_w) +
            " right=0 top=0 bottom=0 ! video/x-raw,format=RGBA,width=" +
            std::to_string(eye_w) + ",height=" + std::to_string(eye_h) +
            " ! glupload ! __stereo_comp__.sink_1 ";
        append_extra_branches(output_chain, "__stereo_comp__", 2, true);
        append_extra_branches(output_chain, "__stereo_comp__",
                              2 + n_extra_streams, false);

        std::string comp_desc =
            "glvideomixer name=__stereo_comp__ background=1"
            " sink_0::xpos=0 sink_0::ypos=0"
            " sink_0::width=" +
            std::to_string(eye_w) + " sink_0::height=" +
            std::to_string(stereo_h) +
            " sink_0::sizing-policy=1"
            " sink_1::xpos=" +
            std::to_string(eye_w) +
            " sink_1::ypos=0"
            " sink_1::width=" +
            std::to_string(eye_w) + " sink_1::height=" +
            std::to_string(stereo_h) + " sink_1::sizing-policy=1";
        const int disp = stereo.display_horizontal_offset_px;
        const int disp_half_lo = disp / 2;
        const int disp_half_hi = disp - disp_half_lo;
        for (int i = 0; i < n_extra_streams; ++i) {
          const int base_x = i * (slot_w + gap_px);
          const int left_x =
              std::max(0, std::min(eye_w - slot_w, base_x - disp_half_lo));
          const int right_x = std::max(
              eye_w, std::min(stereo_w - slot_w,
                              eye_w + base_x + disp_half_hi));
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
          comp_desc +=
              " sink_" + std::to_string(2 + n_extra_streams + i) +
              "::xpos=" + std::to_string(right_x) +
              " sink_" + std::to_string(2 + n_extra_streams + i) +
              "::ypos=" + std::to_string(stereo_h + gap_px) +
              " sink_" + std::to_string(2 + n_extra_streams + i) +
              "::width=" + std::to_string(slot_w) +
              " sink_" + std::to_string(2 + n_extra_streams + i) +
              "::height=" + std::to_string(mono_h) +
              " sink_" + std::to_string(2 + n_extra_streams + i) +
              "::sizing-policy=1" +
              " sink_" + std::to_string(2 + n_extra_streams + i) +
              "::yalign=0.0";
        }
        output_chain += " " + comp_desc;
        output_chain += " ! video/x-raw(memory:GLMemory),width=" +
                        std::to_string(stereo_w) + ",height=" +
                        std::to_string(eye_h) + " ! glcolorconvert";
      } else {
        if (include_overlay) {
          output_chain += " ! gldownload ! videoconvert ! cairooverlay "
                          "name=stereo_overlay ! glupload";
        }
        output_chain += " ! glcolorconvert";
      }

      if (has_glimage || has_glimages) {
        output_chain += " ! tee name=__stereo_display_out__";
      }

      if (has_glimage) {
        output_chain +=
            " __stereo_display_out__. ! queue max-size-buffers=1 "
            "leaky=downstream ! gtkglsink sync=false "
            "force-aspect-ratio=false name=__stereo_sink__";
      }

      if (has_glimages) {
        output_chain +=
            " __stereo_display_out__. ! queue max-size-buffers=1 "
            "leaky=downstream ! gldownload ! videoconvert "
            "! video/x-raw,format=RGBA,width=" +
            std::to_string(stereo_w) + ",height=" + std::to_string(eye_h) +
            " ! tee name=__stereo_eye_split__"
            " __stereo_eye_split__. ! queue max-size-buffers=1 "
            "leaky=downstream ! videocrop left=0 right=" +
            std::to_string(eye_w) +
            " top=0 bottom=0 ! video/x-raw,format=RGBA,width=" +
            std::to_string(eye_w) + ",height=" + std::to_string(eye_h) +
            " ! glupload ! glcolorconvert ! gtkglsink "
            "name=__left_eye_sink__ sync=false force-aspect-ratio=false"
            " __stereo_eye_split__. ! queue max-size-buffers=1 "
            "leaky=downstream ! videocrop left=" +
            std::to_string(eye_w) +
            " right=0 top=0 bottom=0 ! video/x-raw,format=RGBA,width=" +
            std::to_string(eye_w) + ",height=" + std::to_string(eye_h) +
            " ! glupload ! glcolorconvert ! gtkglsink "
            "name=__right_eye_sink__ sync=false force-aspect-ratio=false";
      }
    }

    int stereo_unixfd_index = 0;
    for (const auto &sink : stereo_unixfd_sinks) {
      const std::string socket_path =
          resolve_unixfd_socket_path(stereo.name, sink);
      if (stereo_branches > 1) {
        output_chain +=
            " __stereo_out__. ! queue max-size-buffers=2 max-size-time=0 "
            "max-size-bytes=0 leaky=downstream ! ";
      } else {
        output_chain +=
            " ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
            "leaky=downstream ! ";
      }
      output_chain +=
          get_unixfd_upload_chain() + " ! queue name=__stereo_unixfd_ts_q" +
          std::to_string(stereo_unixfd_index++) +
          "__ max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
          "leaky=downstream ! unixfdsink socket-path=" +
          socket_path + " socket-type=abstract sync=false async=false";
    }

    if (!overlay_unixfd_sinks.empty()) {
      if (stereo_branches > 1) {
        output_chain +=
            " __stereo_out__. ! queue name=__stereo_overlay_input_ts_q__ "
            "max-size-buffers=1 leaky=downstream ! ";
      } else {
        output_chain +=
            " ! queue name=__stereo_overlay_input_ts_q__ "
            "max-size-buffers=1 leaky=downstream ! ";
      }
      output_chain +=
          "gldownload ! videoconvert ! cairooverlay "
          "name=stereo_overlay_unixfd ";

      if (overlay_unixfd_sinks.size() > 1) {
        output_chain += "! tee name=__overlay_out__ ";
      }

      int overlay_unixfd_index = 0;
      for (const auto &sink : overlay_unixfd_sinks) {
        const std::string socket_path =
            resolve_unixfd_socket_path(stereo.name, sink);
        if (overlay_unixfd_sinks.size() > 1) {
          output_chain +=
              " __overlay_out__. ! queue max-size-buffers=2 "
              "max-size-time=0 max-size-bytes=0 leaky=downstream ! ";
        } else {
          output_chain += " ! ";
        }
        output_chain +=
            "videoconvert ! video/x-raw,format=I420"
            " ! queue name=__overlay_unixfd_ts_q" +
            std::to_string(overlay_unixfd_index++) +
            "__ max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
            "leaky=downstream ! unixfdsink socket-path=" +
            socket_path + " socket-type=abstract sync=false async=false";
      }
    }
  } else {
    if (include_overlay) {
      output_chain +=
          " ! gldownload ! videoconvert ! cairooverlay name=stereo_overlay";
    }
    output_chain += " ! fakesink sync=false";
  }

  return extra_chains + input_chain + " " + output_chain;
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

    m_fps_label.set_halign(Gtk::ALIGN_CENTER);
    m_vbox.pack_start(m_fps_label, Gtk::PACK_SHRINK);

    m_fps_timer = Glib::signal_timeout().connect(
        sigc::mem_fun(*this, &ControlWindow::on_update_fps), 500);

    m_btn_overlay.set_label("Overlay");
    m_btn_overlay.set_active(true);
    m_btn_overlay.signal_toggled().connect(
        sigc::mem_fun(*this, &ControlWindow::on_overlay_toggled));
    m_vbox.pack_start(m_btn_overlay, Gtk::PACK_SHRINK);
    m_vbox.pack_start(m_display_outputs.widget(), Gtk::PACK_SHRINK);

    if (!m_cfg.extra_streams.monos.empty() || !m_cfg.extra_streams.stereos.empty()) {
      m_scale_label.set_text("Extra Streams:");
      m_scale_label.set_halign(Gtk::ALIGN_START);
      m_extra_box.pack_start(m_scale_label, Gtk::PACK_SHRINK);
      
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
      m_scale_slider.set_size_request(150, -1);
      m_scale_slider.signal_button_release_event().connect(
          [this](GdkEventButton *) -> bool {
            on_scale_released();
            return false;
          });
      m_extra_box.pack_start(m_scale_slider, Gtk::PACK_EXPAND_WIDGET);
      m_vbox.pack_start(m_extra_box, Gtk::PACK_SHRINK);
      m_scale_visible = true;
    }

    m_btn_quit.set_label("Quit");
    m_btn_quit.signal_clicked().connect(
        sigc::mem_fun(*this, &ControlWindow::on_quit_clicked));
    m_vbox.pack_start(m_btn_quit, Gtk::PACK_SHRINK);

    const int height = 180 + (m_scale_visible ? 35 : 0);
    set_default_size(260, height);
    add_events(Gdk::KEY_PRESS_MASK);
    show_all_children();
    setup_display_windows(pipeline);
  }

  ~ControlWindow() override {
    m_fps_timer.disconnect();
  }

  void attach_sink_probe(GstElement *pipeline, const std::string &sink_name, FpsTracker *tracker) {
    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), sink_name.c_str());
    if (sink) {
      GstPad *pad = gst_element_get_static_pad(sink, "sink");
      if (pad) {
        gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
                          sink_fps_probe_cb, tracker, nullptr);
        gst_object_unref(pad);
      }
      gst_object_unref(sink);
    }
  }

  void setup_display_windows(GstElement *pipeline) {
    m_pipeline = pipeline;
    m_has_display_sinks = false;
    
    // Attach pad probes to measure FPS/Hz
    attach_sink_probe(pipeline, "__left_eye_sink__", &m_left_tracker);
    attach_sink_probe(pipeline, "__right_eye_sink__", &m_right_tracker);
    attach_sink_probe(pipeline, "__stereo_sink__", &m_stereo_tracker);

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
    m_has_display_sinks = !sinks.empty();
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

  bool on_update_fps() {
    std::string fps_text = "Rendering rate (CPU counters): ";
    bool any = false;
    
    auto format_fps = [](double fps) -> std::string {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(1) << fps;
      return ss.str();
    };

    if (m_stereo_tracker.get_count() > 0) {
      fps_text += " Stereo: " + format_fps(m_stereo_tracker.get_fps()) + " Hz";
      any = true;
    }
    if (m_left_tracker.get_count() > 0) {
      if (any) fps_text += " |";
      fps_text += " Left: " + format_fps(m_left_tracker.get_fps()) + " Hz";
      any = true;
    }
    if (m_right_tracker.get_count() > 0) {
      if (any) fps_text += " |";
      fps_text += " Right: " + format_fps(m_right_tracker.get_fps()) + " Hz";
      any = true;
    }

    if (!any && m_has_display_sinks) {
      fps_text += " Waiting for active sinks...";
    } else if (!any) {
      fps_text += " No display sinks configured";
    }
    
    m_fps_label.set_markup("<span weight='bold' size='medium'>" + fps_text + "</span>");
    return true;
  }

  std::shared_ptr<sv::OverlayState> m_overlay_state;
  GstElement *m_pipeline;
  const sv::AppConfig &m_cfg;
  RebuildCb m_rebuild_cb;
  bool m_scale_visible = false;
  bool m_has_display_sinks = false;
  Gtk::Box m_vbox;
  Gtk::Box m_extra_box{Gtk::ORIENTATION_HORIZONTAL, 8};
  Gtk::ToggleButton m_btn_overlay;
  Gtk::Button m_btn_quit;
  Gtk::Label m_scale_label;
  Gtk::Scale m_scale_slider{Gtk::ORIENTATION_HORIZONTAL};
  sv::DisplayOutputPanel m_display_outputs;
  Gtk::Label m_fps_label;
  FpsTracker m_left_tracker;
  FpsTracker m_right_tracker;
  FpsTracker m_stereo_tracker;
  sigc::connection m_fps_timer;
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

  auto node = std::make_shared<rclcpp::Node>("dvrk_console");
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

  if (!sv::Config::check_type(root, "dvrk_console:stereo_display@1.0.0", path)) {
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
  if (cfg.original_width <= 0 || cfg.original_height <= 0) {
    RCLCPP_ERROR(node->get_logger(),
                 "Config '%s' must provide positive "
                 "per-eye width and height via camera.size, "
                 "stereo.eye_size, or stereo.size",
                 cfg.name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  // Populate cfg.stereo.source from unixfdsources "stereo" entry when
  // stereo.stream was not set explicitly in the config.
  if (cfg.stereo.source.empty()) {
    for (const auto &src : cfg.unixfd_sources) {
      if (src.name != "stereo") continue;
      const char *username = getenv("USER");
      if (!username) {
        struct passwd *pw = getpwuid(getuid());
        username = pw ? pw->pw_name : "unknown";
      }
      const std::string socket_path = src.socket_path.empty()
          ? "/tmp/" + cfg.name + "_" + src.name + "_" + std::string(username) + ".sock"
          : src.socket_path;
      cfg.stereo.source = "unixfdsrc socket-path=" + socket_path +
          " socket-type=abstract do-timestamp=true ! video/x-raw,format=I420,width=" +
          std::to_string(2 * cfg.original_width) +
          ",height=" + std::to_string(cfg.original_height);
      RCLCPP_INFO(node->get_logger(),
                  "Stereo source: %s", socket_path.c_str());
      break;
    }
    if (cfg.stereo.source.empty()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Config '%s' must define either stereo.stream or a "
                   "unixfdsources entry with name \"stereo\"",
                   cfg.name.c_str());
      rclcpp::shutdown();
      return 1;
    }
  }

  const sv::AppConfig &app_cfg = cfg;

  warn_missing_unixfd_source_sockets(app_cfg, node->get_logger());

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
                         "dvrk_console_pipeline")) {
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "GStreamer pipeline string:\n%s", pipeline_string.c_str());

  dc_stereo::PipelineUserData pipeline_user_data;
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

  pipeline_user_data.node = node.get();
  pipeline_user_data.reconnector.start(pipeline, node.get(), "stereo_display");

  GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  gst_bus_add_watch(bus, dc_stereo::on_bus_message, &pipeline_user_data);
  gst_object_unref(bus);

  FrameTimestampState timestamp_state;
  attach_timestamp_probes(pipeline, cfg, &timestamp_state);
  attach_ar_timestamp_probes(pipeline);

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

    pipeline_user_data.reconnector.stop();
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
    pipeline_user_data.reconnector.start(pipeline, node.get(), "stereo_display");
    GstBus *new_bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    gst_bus_add_watch(new_bus, dc_stereo::on_bus_message, &pipeline_user_data);
    gst_object_unref(new_bus);

    // Re-attach overlay signals
    attach_overlays(pipeline);
    attach_timestamp_probes(pipeline, cfg, &timestamp_state);
    attach_ar_timestamp_probes(pipeline);

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

  pipeline_user_data.reconnector.stop();

  if (pipeline) {
    gst_element_send_event(pipeline, gst_event_new_eos());
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
  }

  g_app.reset();

  rclcpp::shutdown();
  return 0;
}

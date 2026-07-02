#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <dvrk_data/config.hpp>
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <dvrk_data/stereo_common.hpp>

namespace {

struct CommandLineOptions {
  std::string config_file;
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

GstPadProbeReturn source_timestamp_probe_cb(GstPad *pad,
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

  DcFrameTimestamps timestamps = dc_buffer_get_frame_timestamps(buf);
  const gint64 now = dc_clock_realtime_ns();
  const int side = GPOINTER_TO_INT(user_data);
  if (side == 0) {
    timestamps.left_source_ts = now;
  } else {
    timestamps.right_source_ts = now;
  }
  dc_buffer_set_frame_timestamps(buf, timestamps);
  return GST_PAD_PROBE_OK;
}

void add_timestamp_probe(GstElement *pipeline, const std::string &element_name,
                         int side) {
  GstElement *element =
      gst_bin_get_by_name(GST_BIN(pipeline), element_name.c_str());
  if (element == nullptr) {
    return;
  }

  GstPad *pad = gst_element_get_static_pad(element, "src");
  if (pad != nullptr) {
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
                      source_timestamp_probe_cb, GINT_TO_POINTER(side),
                      nullptr);
    gst_object_unref(pad);
  }
  gst_object_unref(element);
}

std::string build_branch(const std::string &source,
                         const std::string &stream_name,
                         const std::string &queue_name,
                         const std::string &tee_name,
                         const std::vector<sv::UnixfdSinkConfig> &sinks,
                         const sv::AppConfig &cfg) {
  std::string branch =
      source + " ! queue name=" + queue_name +
      " max-size-buffers=3 max-size-time=0 max-size-bytes=0 leaky=downstream";

  if (sinks.size() > 1) {
    branch += " ! tee name=" + tee_name + " ";
  }

  for (std::size_t i = 0; i < sinks.size(); ++i) {
    if (sinks.size() > 1) {
      branch += " " + tee_name +
                ". ! queue max-size-buffers=2 max-size-time=0 "
                "max-size-bytes=0 leaky=downstream";
    }

    const std::string socket_path =
        dc_stereo::resolve_unixfd_socket_path(cfg.name, sinks[i]);
    branch += " ! videoconvert ! video/x-raw,format=I420"
              " ! queue name=__" +
              stream_name + "_unixfd_q" + std::to_string(i) +
              "__ max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
              "leaky=downstream ! unixfdsink socket-path=" +
              socket_path + " socket-type=abstract sync=false async=false";
  }

  return branch;
}

std::string build_pipeline_string(
    const sv::AppConfig &cfg,
    const std::vector<sv::UnixfdSinkConfig> &left_sinks,
    const std::vector<sv::UnixfdSinkConfig> &right_sinks) {
  return build_branch(cfg.left.source, "left", "__left_src_q__", "__left_out__",
                      left_sinks, cfg) +
         " " +
         build_branch(cfg.right.source, "right", "__right_src_q__",
                      "__right_out__", right_sinks, cfg);
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

  auto node = std::make_shared<rclcpp::Node>("stereo_source");
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

  if (!sv::Config::check_type(root, "dvrk_data:stereo_source@1.0.0", path)) {
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

  if (cfg.left.source.empty() || cfg.right.source.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Config '%s' must define camera.left.stream and "
                 "camera.right.stream",
                 cfg.name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  auto left_sinks = dc_stereo::collect_unixfd_sinks(cfg, "left");
  auto right_sinks = dc_stereo::collect_unixfd_sinks(cfg, "right");
  dc_stereo::ensure_sink(left_sinks, "left");
  dc_stereo::ensure_sink(right_sinks, "right");

  dc_stereo::remove_stale_sockets(cfg.name, left_sinks, node->get_logger());
  dc_stereo::remove_stale_sockets(cfg.name, right_sinks, node->get_logger());

  for (const auto &sink : left_sinks) {
    RCLCPP_INFO(node->get_logger(), "left unixfd sink: %s",
                dc_stereo::resolve_unixfd_socket_path(cfg.name, sink).c_str());
  }
  for (const auto &sink : right_sinks) {
    RCLCPP_INFO(node->get_logger(), "right unixfd sink: %s",
                dc_stereo::resolve_unixfd_socket_path(cfg.name, sink).c_str());
  }

  dc_stereo::warn_if_interlaced_stream(cfg.left.source, node->get_logger(),
                                       "camera.left");
  dc_stereo::warn_if_interlaced_stream(cfg.right.source, node->get_logger(),
                                       "camera.right");

  const std::string pipeline_string =
      build_pipeline_string(cfg, left_sinks, right_sinks);
  if (!dc_stereo::validate_pipeline(pipeline_string, node->get_logger(),
                                    "stereo_source")) {
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

  add_timestamp_probe(pipeline, "__left_src_q__", 0);
  add_timestamp_probe(pipeline, "__right_src_q__", 1);

  const int rc = dc_stereo::run_pipeline(
      pipeline, node, "Stereo source unixfd pipeline started");
  gst_object_unref(pipeline);
  rclcpp::shutdown();
  return rc;
}

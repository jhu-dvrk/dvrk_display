#ifndef DVRK_DATA_STEREO_COMMON_HPP
#define DVRK_DATA_STEREO_COMMON_HPP

#include <glib-unix.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <pwd.h>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

#include <dvrk_data/config.hpp>

namespace dc_stereo {

inline GMainLoop *g_main_loop = nullptr;

struct PipelineReconnector {
  GstElement *pipeline = nullptr;
  rclcpp::Node *node = nullptr;
  guint timer_id = 0;
  bool is_active = false;
  std::string name;

  void start(GstElement *pipe, rclcpp::Node *n, const std::string &pipeline_name) {
    pipeline = pipe;
    node = n;
    name = pipeline_name;
    is_active = true;
  }

  void stop() {
    if (timer_id != 0) {
      g_source_remove(timer_id);
      timer_id = 0;
    }
    is_active = false;
    pipeline = nullptr;
  }

  void handle_error_or_eos() {
    if (!is_active || !pipeline) return;

    gst_element_set_state(pipeline, GST_STATE_NULL);

    if (timer_id == 0) {
      RCLCPP_WARN(node->get_logger(), "[%s] Connection lost. Retrying to reconnect...", name.c_str());
      timer_id = g_timeout_add(1000, [](gpointer data) -> gboolean {
        auto *self = static_cast<PipelineReconnector *>(data);
        if (!self->is_active || !self->pipeline) {
          self->timer_id = 0;
          return FALSE;
        }

        GstStateChangeReturn ret = gst_element_set_state(self->pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
          return TRUE;
        }

        RCLCPP_INFO(self->node->get_logger(), "[%s] Successfully reconnected!", self->name.c_str());
        self->timer_id = 0;
        return FALSE;
      }, this);
    }
  }
};

struct PipelineUserData {
  rclcpp::Node *node = nullptr;
  PipelineReconnector reconnector;
};

inline void warn_if_interlaced_stream(const std::string &stream,
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

inline std::string resolve_unixfd_socket_path(
    const std::string &app_name, const sv::UnixfdSinkConfig &sink) {
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

  return "/tmp/" + app_name + "_" + suffix + "_" + std::string(username) +
         ".sock";
}

inline std::vector<sv::UnixfdSinkConfig>
collect_unixfd_sinks(const sv::AppConfig &cfg, const std::string &stream) {
  std::vector<sv::UnixfdSinkConfig> sinks;
  for (const auto &sink : cfg.unixfd_sinks) {
    if (sink.stream == stream) {
      sinks.push_back(sink);
    }
  }
  return sinks;
}

inline std::string resolve_unixfd_source_path(
    const std::string &app_name, const sv::UnixfdSourceConfig &source) {
  if (!source.socket_path.empty()) {
    return source.socket_path;
  }

  const char *username = getenv("USER");
  if (!username) {
    struct passwd *pw = getpwuid(getuid());
    username = pw ? pw->pw_name : "unknown";
  }

  return "/tmp/" + app_name + "_" + source.name + "_" + std::string(username) +
         ".sock";
}

inline std::vector<sv::UnixfdSourceConfig>
collect_unixfd_sources(const sv::AppConfig &cfg, const std::string &name) {
  std::vector<sv::UnixfdSourceConfig> sources;
  for (const auto &src : cfg.unixfd_sources) {
    if (src.name == name) {
      sources.push_back(src);
    }
  }
  return sources;
}

inline std::string build_unixfdsrc_string(const std::string &socket_path,
                                          int width, int height) {
  return "unixfdsrc socket-path=" + socket_path +
         " socket-type=abstract do-timestamp=true"
         " ! video/x-raw,format=I420,width=" +
         std::to_string(width) + ",height=" + std::to_string(height);
}

inline void ensure_sink(std::vector<sv::UnixfdSinkConfig> &sinks,
                        const std::string &stream) {
  if (!sinks.empty()) {
    return;
  }
  sv::UnixfdSinkConfig sink;
  sink.stream = stream;
  sinks.push_back(std::move(sink));
}

inline void remove_stale_sockets(const std::string &app_name,
                                 const std::vector<sv::UnixfdSinkConfig> &sinks,
                                 const rclcpp::Logger &logger) {
  for (const auto &sink : sinks) {
    const std::string socket_path = resolve_unixfd_socket_path(app_name, sink);
    if (!std::filesystem::exists(socket_path)) {
      continue;
    }
    RCLCPP_INFO(logger, "Removing stale unixfd socket before start: %s",
                socket_path.c_str());
    std::error_code remove_error;
    if (!std::filesystem::remove(socket_path, remove_error) && remove_error) {
      RCLCPP_WARN(logger, "Unable to remove stale unixfd socket '%s': %s",
                  socket_path.c_str(), remove_error.message().c_str());
    }
  }
}

inline bool validate_pipeline(const std::string &pipeline_string,
                              const rclcpp::Logger &logger,
                              const std::string &name) {
  GError *error = nullptr;
  GstElement *pipeline = gst_parse_launch(pipeline_string.c_str(), &error);
  if (error == nullptr && pipeline != nullptr) {
    gst_object_unref(pipeline);
    return true;
  }

  RCLCPP_ERROR(logger, "Unable to parse GStreamer pipeline '%s': %s",
               name.c_str(),
               error && error->message ? error->message : "unknown error");
  if (error != nullptr) {
    g_error_free(error);
  }
  if (pipeline != nullptr) {
    gst_object_unref(pipeline);
  }
  return false;
}

inline gboolean on_sigint(gpointer) {
  if (g_main_loop != nullptr) {
    g_main_loop_quit(g_main_loop);
  }
  return G_SOURCE_REMOVE;
}

inline gboolean on_ros_spin(gpointer user_data) {
  if (user_data == nullptr || !rclcpp::ok()) {
    return G_SOURCE_CONTINUE;
  }

  auto *node = static_cast<rclcpp::Node *>(user_data);
  rclcpp::spin_some(node->get_node_base_interface());
  return G_SOURCE_CONTINUE;
}

inline gboolean on_bus_message(GstBus *, GstMessage *msg, gpointer user_data) {
  if (msg == nullptr || user_data == nullptr) {
    return G_SOURCE_CONTINUE;
  }

  auto *data = static_cast<PipelineUserData *>(user_data);
  auto *node = data->node;

  if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
    RCLCPP_INFO(node->get_logger(), "GStreamer bus: received EOS");
    data->reconnector.handle_error_or_eos();
  } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
    GError *err = nullptr;
    gchar *dbg = nullptr;
    gst_message_parse_error(msg, &err, &dbg);
    RCLCPP_ERROR(node->get_logger(), "GStreamer error: %s",
                 err && err->message ? err->message : "unknown");
    if (dbg != nullptr) {
      RCLCPP_ERROR(node->get_logger(), "Debug details: %s", dbg);
      g_free(dbg);
    }
    if (err != nullptr) {
      g_error_free(err);
    }
    data->reconnector.handle_error_or_eos();
  } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_WARNING) {
    GError *err = nullptr;
    gchar *dbg = nullptr;
    gst_message_parse_warning(msg, &err, &dbg);
    RCLCPP_WARN(node->get_logger(), "GStreamer warning: %s",
                 err && err->message ? err->message : "unknown");
    if (dbg != nullptr) {
      RCLCPP_WARN(node->get_logger(), "Debug details: %s", dbg);
      g_free(dbg);
    }
    if (err != nullptr) {
      g_error_free(err);
    }
  }

  return G_SOURCE_CONTINUE;
}

inline int run_pipeline(GstElement *pipeline,
                        const std::shared_ptr<rclcpp::Node> &node,
                        const std::string &started_message) {
  PipelineUserData user_data;
  user_data.node = node.get();
  user_data.reconnector.start(pipeline, node.get(), "stereo_pipeline");

  GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  gst_bus_add_watch(bus, on_bus_message, &user_data);
  gst_object_unref(bus);

  g_unix_signal_add(SIGINT, on_sigint, nullptr);
  g_unix_signal_add(SIGTERM, on_sigint, nullptr);
  g_timeout_add(20, on_ros_spin, node.get());

  g_main_loop = g_main_loop_new(nullptr, FALSE);
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  RCLCPP_INFO(node->get_logger(), "%s", started_message.c_str());
  g_main_loop_run(g_main_loop);

  user_data.reconnector.stop();

  g_main_loop_unref(g_main_loop);
  g_main_loop = nullptr;

  gst_element_send_event(pipeline, gst_event_new_eos());
  gst_element_set_state(pipeline, GST_STATE_NULL);
  return 0;
}

}  // namespace dc_stereo

#endif  // DVRK_DATA_STEREO_COMMON_HPP

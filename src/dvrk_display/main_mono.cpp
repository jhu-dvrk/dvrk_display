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
#include <pwd.h>
#include <string>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "config.hpp"
#include "overlay.hpp"
#include <data_collection/cpu_timestamp_meta.hpp>

namespace {

struct CommandLineOptions {
  std::string config_file;
};


static Glib::RefPtr<Gtk::Application> g_app;







void print_usage(const char *executable) {
  std::cerr << "Usage: " << executable << " -c <config.json>" << std::endl;
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

GstPadProbeReturn source_timestamp_probe_cb(GstPad *pad, GstPadProbeInfo *info,
                                            gpointer user_data) {
  (void)pad;
  (void)user_data;
  if (info->type & GST_PAD_PROBE_TYPE_BUFFER) {
    GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!gst_buffer_is_writable(buf)) {
      buf = gst_buffer_make_writable(buf);
      GST_PAD_PROBE_INFO_DATA(info) = buf;
    }

    if (!gst_buffer_get_custom_meta(buf, DC_CPU_TIMESTAMP_META_NAME)) {
      dc_buffer_add_cpu_timestamp(buf, dc_clock_realtime_ns());
    }
  }
  return GST_PAD_PROBE_OK;
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

gboolean on_bus_message(GstBus *, GstMessage *msg, gpointer) {
  if (msg == nullptr) {
    return G_SOURCE_CONTINUE;
  }

  if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
    if (g_app) {
      g_app->quit();
    }
  } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
    GError *err = nullptr;
    gchar *dbg = nullptr;
    gst_message_parse_error(msg, &err, &dbg);
    std::cerr << "GStreamer error: " << (err ? err->message : "unknown")
              << std::endl;
    if (dbg != nullptr) {
      std::cerr << "Debug details: " << dbg << std::endl;
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




std::string build_pipeline_string(const sv::AppConfig &app_cfg, bool overlay_available) {
  std::string p;

  p += app_cfg.stream + " ";
  p += "! tee name=__raw_out__ ";

  for (const auto &sink : app_cfg.unixfd_sinks) {
    if (sink.stream == "raw") {
      p += "__raw_out__. ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! video/x-raw,format=I420 ! ";
      std::string sock = resolve_unixfd_socket_path(app_cfg.name, sink);
      p += "unixfdsink socket-path=" + sock + " ";
      if (!sink.name.empty()) {
        p += "name=" + sink.name + " ";
      }
      p += "sync=false async=false ";
    }
  }

  p += "__raw_out__. ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! ";

  if (overlay_available) {
    p += "cairooverlay name=mono_overlay ! videoconvert ! ";
  }

  p += "tee name=__overlay_out__ ";

  for (const auto &sink : app_cfg.unixfd_sinks) {
    if (sink.stream == "overlay") {
      p += "__overlay_out__. ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! video/x-raw,format=I420 ! ";
      std::string sock = resolve_unixfd_socket_path(app_cfg.name, sink);
      p += "unixfdsink socket-path=" + sock + " ";
      if (!sink.name.empty()) {
        p += "name=" + sink.name + " ";
      }
      p += "sync=false async=false ";
    }
  }

  for (size_t i = 0; i < app_cfg.sinks.size(); ++i) {
    std::string s = app_cfg.sink_streams[i];
    // Rename gtkglsink to __mono_sink__ so we can get its widget
    if (s.find("gtkglsink") != std::string::npos) {
      if (s.find("name=") != std::string::npos) {
         // replace the old name
      } else {
         s += " name=__mono_sink__";
      }
    } else if (s.find("glimagesink") != std::string::npos) {
         // use gtkglsink instead to embed
         s = "glupload ! glcolorconvert ! gtkglsink name=__mono_sink__ sync=false force-aspect-ratio=false";
    }
    p += "__overlay_out__. ! queue max-size-buffers=2 leaky=downstream ! " + s + " ";
  }

  return p;
}

class ControlWindow : public Gtk::Window {
public:
  using RebuildCb = std::function<void(double new_scale)>;

  ControlWindow(std::shared_ptr<sv::OverlayState> overlay_state,
                GstElement *pipeline,
                const sv::AppConfig &cfg,
                RebuildCb rebuild_cb)
      : m_overlay_state(overlay_state), m_pipeline(pipeline),
        m_cfg(cfg), m_rebuild_cb(std::move(rebuild_cb)) {
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

    m_btn_fullscreen.set_label("Fullscreen");
    m_btn_fullscreen.set_active(false);
    m_btn_fullscreen.signal_toggled().connect(
        sigc::mem_fun(*this, &ControlWindow::on_fullscreen_toggled));
    m_vbox.pack_start(m_btn_fullscreen, Gtk::PACK_SHRINK);

    m_btn_quit.set_label("Quit");
    m_btn_quit.signal_clicked().connect(
        sigc::mem_fun(*this, &ControlWindow::on_quit_clicked));
    m_vbox.pack_start(m_btn_quit, Gtk::PACK_SHRINK);

    set_default_size(260, 120);
    add_events(Gdk::KEY_PRESS_MASK);
    show_all_children();
    setup_display_windows(pipeline);
  }

  void setup_display_windows(GstElement *pipeline) {
    m_pipeline = pipeline;
    m_display_windows.clear();
    
    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "__mono_sink__");
    if (!sink) return;
    
    GtkWidget *gtk_widget = nullptr;
    g_object_get(sink, "widget", &gtk_widget, NULL);
    gst_object_unref(sink);
    if (!gtk_widget) return;
    
    auto win = std::make_unique<Gtk::Window>();
    win->add_events(Gdk::KEY_PRESS_MASK);
    win->signal_key_press_event().connect(
        [this](GdkEventKey *event) {
          if ((event->state & GDK_CONTROL_MASK) &&
              (event->keyval == GDK_KEY_q)) {
            on_quit_clicked();
            return true;
          }
          return false;
        },
        false);
    win->set_title(m_cfg.name + " (Mono)");
    win->set_default_size(m_cfg.original_width > 0 ? m_cfg.original_width : 640,
                          m_cfg.original_height > 0 ? m_cfg.original_height : 480);
    Gtk::Widget *mm_widget = Glib::wrap(gtk_widget);
    win->add(*mm_widget);
    win->show_all();
    if (m_btn_fullscreen.get_active()) win->fullscreen();
    m_display_windows.push_back(std::make_pair("mono", std::move(win)));
    g_object_unref(gtk_widget);
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

  void on_fullscreen_toggled() {
    bool active = m_btn_fullscreen.get_active();
    for (auto &pair : m_display_windows) {
      if (active) pair.second->fullscreen();
      else        pair.second->unfullscreen();
    }
  }

  void on_quit_clicked() {
    if (g_app) g_app->quit();
  }

  std::shared_ptr<sv::OverlayState> m_overlay_state;
  GstElement *m_pipeline;
  const sv::AppConfig &m_cfg;
  RebuildCb m_rebuild_cb;
  Gtk::Box m_vbox;
  Gtk::ToggleButton m_btn_overlay;
  Gtk::ToggleButton m_btn_fullscreen;
  Gtk::Button m_btn_quit;
  std::vector<std::pair<std::string, std::unique_ptr<Gtk::Window>>>
      m_display_windows;
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

  if (!sv::Config::check_type(root, "dd::mono_config@1.0.0", path)) {
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
  overlay_state->display_horizontal_offset_px =
      cfg.display_horizontal_offset_px;
  const sv::AppConfig &app_cfg = cfg;

  if (app_cfg.stream.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Config '%s' must define stream",
                 cfg.name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  warn_if_interlaced_stream(app_cfg.stream, node->get_logger(), "stream");
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

  RCLCPP_INFO(node->get_logger(), "Generated pipeline:\n%s", pipeline_string.c_str());
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
  gst_bus_add_watch(bus, on_bus_message, nullptr);
  gst_object_unref(bus);

  GstElement *unixfd_q =
      gst_bin_get_by_name(GST_BIN(pipeline), "__unixfd_ts_q__");
  if (unixfd_q) {
    GstPad *pad = gst_element_get_static_pad(unixfd_q, "src");
    if (pad != nullptr) {
      gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
                        source_timestamp_probe_cb, nullptr, nullptr);
      gst_object_unref(pad);
    }
    gst_object_unref(unixfd_q);
  }

  // Helper: attach cairo overlay signals to all overlay elements in a pipeline.
  // Used for both initial setup and after pipeline rebuild.
  bool first_overlay_found = false;
  auto attach_overlays = [&](GstElement *pl) {
    if (!overlay_available) return;
    const std::vector<std::string> overlay_names = {"mono_overlay"};
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


  g_app = Gtk::Application::create("org.dvrk.display");
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
    gst_bus_add_watch(new_bus, on_bus_message, nullptr);
    gst_object_unref(new_bus);

    // Re-attach overlay signals
    attach_overlays(pipeline);

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
  RCLCPP_INFO(node->get_logger(), "Mono display pipeline started");

  g_app->run(window);

  RCLCPP_INFO(node->get_logger(), "Mono display pipeline on quit: %s",
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


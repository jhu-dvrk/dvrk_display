#include <geometry_msgs/msg/pose_stamped.hpp>
#include <glib-unix.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>

#include <gtkmm.h>
#include <gst/video/navigation.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
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

enum class RosImageTarget { Left, Right, Stereo };

struct RosImagePublisherContext {
  RosImageTarget target = RosImageTarget::Stereo;
  std::string topic_base;
  std::string frame_id;
  image_transport::CameraPublisher publisher;
};

static Glib::RefPtr<Gtk::Application> g_app;
static rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr
    g_timestamp_pub = nullptr;
static std::atomic<std::uint64_t> g_unixfd_offset_counter{0};

typedef struct _CpuTimestampMeta {
  GstMeta meta;
  gint64 cpu_timestamp_ns;
} CpuTimestampMeta;

GType cpu_timestamp_meta_api_get_type(void) {
  static GType type = 0;
  static const gchar *tags[] = {"cpu_timestamp", nullptr};
  if (g_once_init_enter(&type)) {
    GType new_type = gst_meta_api_type_register("CpuTimestampMetaAPI", tags);
    g_once_init_leave(&type, new_type);
  }
  return type;
}

const GstMetaInfo *cpu_timestamp_meta_get_info(void) {
  static const GstMetaInfo *meta_info = nullptr;
  if (g_once_init_enter(&meta_info)) {
    const GstMetaInfo *new_meta_info = gst_meta_register(
        cpu_timestamp_meta_api_get_type(), "CpuTimestampMeta",
        sizeof(CpuTimestampMeta),
        [](GstMeta *meta, gpointer, GstBuffer *) -> gboolean {
          reinterpret_cast<CpuTimestampMeta *>(meta)->cpu_timestamp_ns = 0;
          return TRUE;
        },
        [](GstMeta *, GstBuffer *) -> void {},
        [](GstBuffer *dest, GstMeta *meta, GstBuffer *, GQuark,
           gpointer) -> gboolean {
          const auto *src_meta = reinterpret_cast<CpuTimestampMeta *>(meta);
          auto *dest_meta = reinterpret_cast<CpuTimestampMeta *>(
              gst_buffer_add_meta(dest, cpu_timestamp_meta_get_info(), nullptr));
          if (dest_meta == nullptr) {
            return FALSE;
          }
          dest_meta->cpu_timestamp_ns = src_meta->cpu_timestamp_ns;
          return TRUE;
        });
    g_once_init_leave(&meta_info, new_meta_info);
  }
  return meta_info;
}

CpuTimestampMeta *gst_buffer_add_cpu_timestamp_meta(GstBuffer *buffer,
                                                    gint64 cpu_timestamp_ns) {
  auto *meta = reinterpret_cast<CpuTimestampMeta *>(
      gst_buffer_add_meta(buffer, cpu_timestamp_meta_get_info(), nullptr));
  if (meta != nullptr) {
    meta->cpu_timestamp_ns = cpu_timestamp_ns;
  }
  return meta;
}

CpuTimestampMeta *gst_buffer_get_cpu_timestamp_meta(GstBuffer *buffer) {
  return reinterpret_cast<CpuTimestampMeta *>(
      gst_buffer_get_meta(buffer, cpu_timestamp_meta_api_get_type()));
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

std::string resolve_unixfd_socket_path(const sv::AppConfig &stereo) {
  if (!stereo.has_unixfd_socket_path) {
    return "";
  }

  if (!stereo.unixfd_socket_path.empty()) {
    return stereo.unixfd_socket_path;
  }

  const char *username = getenv("USER");
  if (!username) {
    struct passwd *pw = getpwuid(getuid());
    username = pw ? pw->pw_name : "unknown";
  }
  return "/tmp/" + stereo.name + "_" + std::string(username) + ".sock";
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

std::string to_lower(std::string value) {
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::string trim_topic_tokens(std::string value) {
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

bool parse_ros_image_target(const std::string &value, RosImageTarget &target) {
  const std::string normalized = to_lower(value);
  if (normalized == "left") {
    target = RosImageTarget::Left;
    return true;
  }
  if (normalized == "right") {
    target = RosImageTarget::Right;
    return true;
  }
  if (normalized == "stereo") {
    target = RosImageTarget::Stereo;
    return true;
  }
  return false;
}

std::string ros_image_target_name(const RosImageTarget target) {
  switch (target) {
  case RosImageTarget::Left:
    return "left";
  case RosImageTarget::Right:
    return "right";
  case RosImageTarget::Stereo:
    return "stereo";
  }
  return "stereo";
}

std::string ros_sink_name(const RosImageTarget target) {
  switch (target) {
  case RosImageTarget::Left:
    return "__ros_left_sink__";
  case RosImageTarget::Right:
    return "__ros_right_sink__";
  case RosImageTarget::Stereo:
    return "__ros_stereo_sink__";
  }
  return "__ros_stereo_sink__";
}

bool has_ros_target(const std::vector<RosImageTarget> &targets,
                    const RosImageTarget target) {
  return std::find(targets.begin(), targets.end(), target) != targets.end();
}

bool parse_ros_image_publishers(const std::vector<std::string> &values,
                                const rclcpp::Logger &logger,
                                std::vector<RosImageTarget> &targets) {
  targets.clear();
  for (const auto &value : values) {
    RosImageTarget target;
    if (!parse_ros_image_target(value, target)) {
      RCLCPP_ERROR(logger,
                   "Invalid ros_image_publishers entry '%s'. Allowed values "
                   "are: left, right, stereo",
                   value.c_str());
      return false;
    }
    if (!has_ros_target(targets, target)) {
      targets.push_back(target);
    }
  }
  return true;
}

GstFlowReturn on_new_ros_image_sample(GstElement *sink, gpointer user_data) {
  auto *publisher_context = static_cast<RosImagePublisherContext *>(user_data);
  if (publisher_context == nullptr) {
    return GST_FLOW_OK;
  }

  GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
  if (sample == nullptr) {
    return GST_FLOW_OK;
  }

  GstBuffer *buffer = gst_sample_get_buffer(sample);
  GstCaps *caps = gst_sample_get_caps(sample);
  GstVideoInfo video_info;

  if (buffer != nullptr && caps != nullptr &&
      gst_video_info_from_caps(&video_info, caps)) {
    GstVideoFrame frame;
    if (gst_video_frame_map(&frame, &video_info, buffer, GST_MAP_READ)) {
      auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
      auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();

      struct timespec current_time;
      clock_gettime(CLOCK_REALTIME, &current_time);
      image_msg->header.stamp.sec = current_time.tv_sec;
      image_msg->header.stamp.nanosec = current_time.tv_nsec;
      image_msg->header.frame_id = publisher_context->frame_id;

      image_msg->height = GST_VIDEO_INFO_HEIGHT(&video_info);
      image_msg->width = GST_VIDEO_INFO_WIDTH(&video_info);
      image_msg->encoding = sensor_msgs::image_encodings::RGB8;
      image_msg->is_bigendian = 0;
      image_msg->step = image_msg->width * 3;

      const size_t image_size = image_msg->step * image_msg->height;
      image_msg->data.resize(image_size);

      const uint8_t *src =
          static_cast<uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
      const int src_stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
      uint8_t *dst = image_msg->data.data();

      if (static_cast<int>(image_msg->step) == src_stride) {
        std::memcpy(dst, src, image_size);
      } else {
        for (uint32_t row = 0; row < image_msg->height; ++row) {
          std::memcpy(dst + row * image_msg->step, src + row * src_stride,
                      image_msg->step);
        }
      }

      camera_info_msg->header = image_msg->header;
      camera_info_msg->height = image_msg->height;
      camera_info_msg->width = image_msg->width;
      camera_info_msg->distortion_model = "plumb_bob";
      camera_info_msg->d.resize(5, 0.0);
      camera_info_msg->k[0] = image_msg->width;
      camera_info_msg->k[2] = image_msg->width / 2.0;
      camera_info_msg->k[4] = image_msg->width;
      camera_info_msg->k[5] = image_msg->height / 2.0;
      camera_info_msg->k[8] = 1.0;

      publisher_context->publisher.publish(image_msg, camera_info_msg);
      gst_video_frame_unmap(&frame);
    }
  }

  gst_sample_unref(sample);
  return GST_FLOW_OK;
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

    long long cpu_timestamp = 0;
    if (auto *meta = gst_buffer_get_cpu_timestamp_meta(buf)) {
      cpu_timestamp = meta->cpu_timestamp_ns;
    }
    if (cpu_timestamp == 0) {
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      cpu_timestamp =
          static_cast<long long>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
      gst_buffer_add_cpu_timestamp_meta(buf, cpu_timestamp);
    }

    const std::uint64_t remote_offset = g_unixfd_offset_counter.fetch_add(1);
    GST_BUFFER_OFFSET(buf) = remote_offset;
    GST_BUFFER_OFFSET_END(buf) = remote_offset + 1;

    if (g_timestamp_pub) {
      auto msg = std_msgs::msg::Header();
      msg.stamp.sec = static_cast<int32_t>((remote_offset >> 32) & 0xffffffffULL);
      msg.stamp.nanosec = static_cast<uint32_t>(remote_offset & 0xffffffffULL);
      msg.frame_id = std::to_string(cpu_timestamp);
      g_timestamp_pub->publish(msg);
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

std::string
build_pipeline_string(const sv::AppConfig &stereo,
                      const std::vector<RosImageTarget> &ros_targets,
                      const bool include_overlay) {
  int base_crop_w = stereo.crop_width > 0 ? stereo.crop_width : stereo.original_width;
  int base_crop_h = stereo.crop_height > 0 ? stereo.crop_height : stereo.original_height;
  base_crop_w = normalize_eye_size_for_even_crop(stereo.original_width, base_crop_w);
  base_crop_h = normalize_eye_size_for_even_crop(stereo.original_height, base_crop_h);

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

    aspect_crop_l = diff_x / 2;
    aspect_crop_r = diff_x - aspect_crop_l;
    aspect_crop_t = diff_y / 2;
    aspect_crop_b = diff_y - aspect_crop_t;
  }

  const int eye_w = stereo.preserve_size ? stereo.original_width : base_crop_w;
  const int eye_h = stereo.preserve_size ? stereo.original_height : base_crop_h;

  const bool publish_left = has_ros_target(ros_targets, RosImageTarget::Left);
  const bool publish_right = has_ros_target(ros_targets, RosImageTarget::Right);
  const bool publish_stereo =
      has_ros_target(ros_targets, RosImageTarget::Stereo);
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

  std::string scale_suffix = "";
  if (stereo.preserve_size && stereo.original_width > 0 &&
      stereo.original_height > 0) {
    scale_suffix = " ! videoscale ! video/x-raw,width=" +
                   std::to_string(stereo.original_width) +
                   ",height=" + std::to_string(stereo.original_height);
  }

  std::string left_chain =
      stereo.left.source + " ! queue name=__left_src_q__ max-size-buffers=8 leaky=downstream " +
      color_adjustment_string(stereo.left_color) +
      " ! videocrop left=" +
      std::to_string(left_crop.left) +
      " right=" + std::to_string(left_crop.right) +
      " top=" + std::to_string(left_crop.top) +
      " bottom=" + std::to_string(left_crop.bottom) + scale_suffix;
  if (publish_left || has_glimages) {
    left_chain += " ! tee name=__left_out__"
                  " __left_out__. ! queue max-size-buffers=1 leaky=downstream "
                  "! glupload ! mix.sink_0";
    if (publish_left) {
      left_chain += " __left_out__. ! queue max-size-buffers=2 max-size-time=0 "
                    "max-size-bytes=0 leaky=downstream"
                    " ! videoconvert ! video/x-raw,format=RGB"
                    " ! appsink name=__ros_left_sink__ sync=false async=false "
                    "emit-signals=true drop=true max-buffers=1";
    }
    if (has_glimages) {
      left_chain +=
          " __left_out__. ! queue max-size-buffers=1 leaky=downstream";
      if (include_overlay) {
        left_chain += " ! videoconvert ! cairooverlay name=left_overlay";
      }
      left_chain += " ! glupload ! glcolorconvert ! gtkglsink name=__left_eye_sink__ "
                    "sync=false force-aspect-ratio=false";
    }
  } else {
    left_chain += " ! glupload ! mix.sink_0";
  }

  std::string right_chain =
      stereo.right.source + " ! queue name=__right_src_q__ max-size-buffers=8 leaky=downstream " +
      color_adjustment_string(stereo.right_color) +
      " ! videocrop left=" +
      std::to_string(right_crop.left) +
      " right=" + std::to_string(right_crop.right) +
      " top=" + std::to_string(right_crop.top) +
      " bottom=" + std::to_string(right_crop.bottom) + scale_suffix;
  if (publish_right || has_glimages) {
    right_chain += " ! tee name=__right_out__"
                   " __right_out__. ! queue max-size-buffers=1 "
                   "leaky=downstream ! glupload ! mix.sink_1";
    if (publish_right) {
      right_chain +=
          " __right_out__. ! queue max-size-buffers=2 max-size-time=0 "
          "max-size-bytes=0 leaky=downstream"
          " ! videoconvert ! video/x-raw,format=RGB"
          " ! appsink name=__ros_right_sink__ sync=false async=false "
          "emit-signals=true drop=true max-buffers=1"; // videoconvert from
                                                       // native YUV to RGB
    }
    if (has_glimages) {
      right_chain +=
          " __right_out__. ! queue max-size-buffers=1 leaky=downstream";
      if (include_overlay) {
        right_chain += " ! videoconvert ! cairooverlay name=right_overlay";
      }
      right_chain += " ! glupload ! glcolorconvert ! gtkglsink name=__right_eye_sink__ "
                     "sync=false force-aspect-ratio=false";
    }
  } else {
    right_chain += " ! glupload ! mix.sink_1";
  }

  const double horizontal_ui_scale = (eye_w > 0) ? (static_cast<double>(eye_w) - 2.0 * std::abs(static_cast<double>(stereo.display_horizontal_offset_px))) / static_cast<double>(eye_w) : 1.0;
  const int shifted_eye_w = static_cast<int>(std::round(static_cast<double>(eye_w) * horizontal_ui_scale));
  const int left_xpos = (eye_w / 2) - stereo.display_horizontal_offset_px / 2 - (shifted_eye_w / 2);
  const int right_xpos = eye_w + (eye_w / 2) + stereo.display_horizontal_offset_px / 2 - (shifted_eye_w / 2);

  std::string output_chain =
      "glvideomixer name=mix background=1 sink_0::xpos=" + std::to_string(left_xpos) +
      " sink_0::width=" + std::to_string(shifted_eye_w) +
      " sink_1::xpos=" + std::to_string(right_xpos) +
      " sink_1::width=" + std::to_string(shifted_eye_w) +
      " ! video/x-raw(memory:GLMemory),width=" + std::to_string(2 * eye_w) +
      ",height=" + std::to_string(eye_h);

  const bool need_stereo_tee =
      has_glimage || stereo.has_unixfd_socket_path || publish_stereo;
  if (need_stereo_tee) {
    output_chain += " ! tee name=__stereo_out__ ";
    if (has_glimage) {
      output_chain +=
          "__stereo_out__. ! queue max-size-buffers=1 leaky=downstream";
      if (include_overlay) {
        output_chain += " ! gldownload ! videoconvert ! cairooverlay "
                        "name=stereo_overlay ! glupload";
      }
      output_chain += " ! glcolorconvert ! gtkglsink sync=false force-aspect-ratio=false name=__stereo_sink__";
    }

    if (stereo.has_unixfd_socket_path) {
      const std::string socket_path = resolve_unixfd_socket_path(stereo);
      const std::string unixfd_upload_chain = get_unixfd_upload_chain();
      output_chain += " __stereo_out__. ! queue max-size-buffers=2 "
                      "max-size-time=0 max-size-bytes=0 leaky=downstream"
                      " ! " +
                      unixfd_upload_chain +
              " ! queue name=__unixfd_ts_q__ max-size-buffers=2 "
              "max-size-time=0 max-size-bytes=0 leaky=downstream"
                      " ! unixfdsink socket-path=" + socket_path +
                      " sync=true async=false";
    }

    if (publish_stereo) {
      output_chain += " __stereo_out__. ! queue max-size-buffers=2 "
                      "max-size-time=0 max-size-bytes=0 leaky=downstream"
                      " ! gldownload ! videoconvert ! video/x-raw,format=RGB"
                      " ! appsink name=__ros_stereo_sink__ sync=false "
                      "async=false emit-signals=true drop=true max-buffers=1";
    }
  } else {
    if (include_overlay) {
      output_chain +=
          " ! gldownload ! videoconvert ! cairooverlay name=stereo_overlay";
    }
    output_chain += " ! fakesink sync=false";
  }

  return left_chain + " " + right_chain + " " + output_chain;
}

class ControlWindow : public Gtk::Window {
public:
  ControlWindow(std::shared_ptr<sv::OverlayState> overlay_state, GstElement* pipeline)
      : m_overlay_state(overlay_state), m_pipeline(pipeline) {
    set_title("dVRK Display Control");
    set_border_width(10);
    set_default_size(250, 120);

    m_vbox.set_orientation(Gtk::ORIENTATION_VERTICAL);
    m_vbox.set_spacing(10);
    add(m_vbox);

    m_btn_overlay.set_label("Overlay On/Off");
    m_btn_overlay.set_active(true);
    m_btn_overlay.signal_toggled().connect(sigc::mem_fun(*this, &ControlWindow::on_overlay_toggled));
    m_vbox.pack_start(m_btn_overlay);

    m_btn_fullscreen.set_label("Fullscreen On/Off");
    m_btn_fullscreen.set_active(false);
    m_btn_fullscreen.signal_toggled().connect(sigc::mem_fun(*this, &ControlWindow::on_fullscreen_toggled));
    m_vbox.pack_start(m_btn_fullscreen);

    show_all_children();

    const std::vector<std::string> sink_names = {"__left_eye_sink__", "__right_eye_sink__", "__stereo_sink__"};
    for (const auto& name : sink_names) {
      GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), name.c_str());
      if (sink) {
        GtkWidget *gtk_widget = nullptr;
        g_object_get(sink, "widget", &gtk_widget, NULL);
        if (gtk_widget) {
          auto win = std::make_unique<Gtk::Window>();
          win->set_title(name == "__stereo_sink__" ? "Stereo Display" : (name == "__left_eye_sink__" ? "Left Eye Display" : "Right Eye Display"));
          win->set_default_size(1280, 720);
          Gtk::Widget* mm_widget = Glib::wrap(gtk_widget);
          win->add(*mm_widget);
          win->show_all();
          m_display_windows.push_back(std::make_pair(name, std::move(win)));
        }
        gst_object_unref(sink);
      }
    }
  }

protected:
  void on_overlay_toggled() {
    std::scoped_lock<std::mutex> lock(m_overlay_state->mutex);
    m_overlay_state->overlay_enabled = m_btn_overlay.get_active();
  }

  void on_fullscreen_toggled() {
    bool active = m_btn_fullscreen.get_active();
    for (auto& pair : m_display_windows) {
      if (pair.first == "__stereo_sink__") {
        if (active) pair.second->fullscreen();
        else pair.second->unfullscreen();
      }
    }
  }

  std::shared_ptr<sv::OverlayState> m_overlay_state;
  GstElement* m_pipeline;
  Gtk::Box m_vbox;
  Gtk::ToggleButton m_btn_overlay;
  Gtk::ToggleButton m_btn_fullscreen;
  std::vector<std::pair<std::string, std::unique_ptr<Gtk::Window>>> m_display_windows;
};

} // namespace

int main(int argc, char *argv[]) {
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

  const sv::AppConfig cfg = sv::Config::parse_app_config(root);

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
  std::string selected_viewer_name = "dvrk_display";
  std::vector<RosImageTarget> selected_ros_targets;

  RCLCPP_INFO(node->get_logger(), "Loaded viewer config: %s", cfg.name.c_str());
  console_name = cfg.dvrk_console_namespace;
  overlay_state->overlay_alpha = cfg.overlay_alpha;
  overlay_state->display_horizontal_offset_px = cfg.display_horizontal_offset_px;
  selected_viewer_name = cfg.name.empty() ? "dvrk_display" : cfg.name;
  const sv::AppConfig &app_cfg = cfg;

  if (!parse_ros_image_publishers(cfg.ros_image_publishers, node->get_logger(),
                                  selected_ros_targets)) {
    rclcpp::shutdown();
    return 1;
  }

  if (!selected_ros_targets.empty()) {
    std::string topics;
    const std::string topic_prefix = trim_topic_tokens(selected_viewer_name);
    for (const auto &target : selected_ros_targets) {
      if (!topics.empty()) {
        topics += ", ";
      }
      topics +=
          topic_prefix + "/" + ros_image_target_name(target) + "/image_raw";
    }
    RCLCPP_INFO(node->get_logger(), "ROS image publishing enabled for: %s",
                topics.c_str());
  }

  if (app_cfg.left.source.empty() || app_cfg.right.source.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Config '%s' must define left_stream and right_stream",
                 cfg.name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  warn_if_interlaced_stream(app_cfg.left.source, node->get_logger(), "left");
  warn_if_interlaced_stream(app_cfg.right.source, node->get_logger(), "right");

  // Initialize timestamp publisher with reliable QoS
  g_timestamp_pub = node->create_publisher<std_msgs::msg::Header>(
      app_cfg.name + "/unixfd_timestamps", rclcpp::QoS(100).reliable());

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
    RCLCPP_WARN(node->get_logger(),
                "Config '%s' has odd crop dimensions (%dx%d). "
                "Cropping will be rounded to even values to prevent display artifacts.",
                cfg.name.c_str(), app_cfg.crop_width, app_cfg.crop_height);
  }

  const std::string unixfd_upload_chain = get_unixfd_upload_chain();
  const std::string unixfd_socket_path = resolve_unixfd_socket_path(app_cfg);
  if (app_cfg.has_unixfd_socket_path) {
    if (app_cfg.unixfd_socket_path.empty()) {
      RCLCPP_INFO(node->get_logger(), "unixfd publish path (default): %s",
                  unixfd_socket_path.c_str());
    } else {
      RCLCPP_INFO(node->get_logger(), "unixfd publish path: %s",
                  unixfd_socket_path.c_str());
    }
    RCLCPP_INFO(node->get_logger(), "unixfd export chain: %s",
                unixfd_upload_chain.c_str());
  } else {
    RCLCPP_INFO(node->get_logger(),
                "unixfd publish disabled (unixfd_socket_path is set to empty)");
  }

  if (app_cfg.sink_streams.empty()) {
    RCLCPP_WARN(node->get_logger(), "Resolved sink_streams list is empty");
  } else {
    for (std::size_t i = 0; i < app_cfg.sink_streams.size(); ++i) {
      RCLCPP_INFO(node->get_logger(), "sink_streams[%zu]: %s", i,
                  app_cfg.sink_streams[i].c_str());
    }
  }

  pipeline_string =
      build_pipeline_string(app_cfg, selected_ros_targets, overlay_available);

  std::vector<std::unique_ptr<RosImagePublisherContext>> ros_publisher_contexts;
  std::shared_ptr<image_transport::ImageTransport> ros_image_transport;
  if (!selected_ros_targets.empty()) {
    ros_image_transport =
        std::make_shared<image_transport::ImageTransport>(node);
    const std::string viewer_topic_base =
        trim_topic_tokens(selected_viewer_name);
    for (const auto &target : selected_ros_targets) {
      auto publisher_context = std::make_unique<RosImagePublisherContext>();
      publisher_context->target = target;
      publisher_context->topic_base =
          viewer_topic_base + "/" + ros_image_target_name(target);
      publisher_context->frame_id = publisher_context->topic_base + "_frame";
      publisher_context->publisher = ros_image_transport->advertiseCamera(
          publisher_context->topic_base + "/image_raw", 10);
      ros_publisher_contexts.push_back(std::move(publisher_context));
    }
  }

  const std::string camera_topic = "/" + console_name + "/camera";
  const std::string clutch_topic = "/" + console_name + "/clutch";
  const std::string operator_present_topic = "/" + console_name + "/operator_present";
  const std::string teleop_selected_topic =
      "/" + console_name + "/teleop/selected";
  const std::string teleop_unselected_topic =
      "/" + console_name + "/teleop/unselected";
  RCLCPP_INFO(node->get_logger(),
              "Console topics: camera=%s clutch=%s operator_present=%s teleop_selected=%s "
              "teleop_unselected=%s",
              camera_topic.c_str(), clutch_topic.c_str(), operator_present_topic.c_str(),
              teleop_selected_topic.c_str(), teleop_unselected_topic.c_str());

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

  auto following_subscribers_cache = std::make_shared<std::unordered_map<
      std::string, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>>();
  auto measured_cp_subscribers_cache = std::make_shared<std::unordered_map<
      std::string,
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>>();
  auto tool_type_subscribers_cache = std::make_shared<std::unordered_map<
      std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr>>();
  auto active_teleops = std::make_shared<std::unordered_set<std::string>>();
  auto latest_teleop_by_mtm =
      std::make_shared<std::unordered_map<std::string, std::string>>();
  auto teleop_selected_sub = node->create_subscription<std_msgs::msg::String>(
      teleop_selected_topic, teleop_selected_qos,
      [node, overlay_state, following_subscribers_cache,
       measured_cp_subscribers_cache, tool_type_subscribers_cache,
       active_teleops, latest_teleop_by_mtm, latch_qos, measured_cp_qos,
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

  if (overlay_available) {
    bool found_overlay = false;
    const std::vector<std::string> overlay_names = {
        "stereo_overlay", "left_overlay", "right_overlay"};

    for (const auto &overlay_name : overlay_names) {
      GstElement *overlay =
          gst_bin_get_by_name(GST_BIN(pipeline), overlay_name.c_str());
      if (overlay == nullptr) {
        continue;
      }

      found_overlay = true;
      g_signal_connect(overlay, "caps-changed",
                       G_CALLBACK(sv::on_overlay_caps_changed),
                       overlay_state.get());
      g_signal_connect(overlay, "draw", G_CALLBACK(sv::on_overlay_draw),
                       overlay_state.get());
      gst_object_unref(overlay);
    }

    if (!found_overlay) {
      RCLCPP_WARN(node->get_logger(),
                  "Unable to find overlay element in pipeline; dVRK status "
                  "overlay is disabled");
    }
  }

  for (auto &publisher_context : ros_publisher_contexts) {
    GstElement *ros_sink = gst_bin_get_by_name(
        GST_BIN(pipeline), ros_sink_name(publisher_context->target).c_str());
    if (ros_sink == nullptr) {
      RCLCPP_ERROR(
          node->get_logger(),
          "Failed to find GStreamer sink '%s' for ROS image target '%s'",
          ros_sink_name(publisher_context->target).c_str(),
          ros_image_target_name(publisher_context->target).c_str());
      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_object_unref(pipeline);
      rclcpp::shutdown();
      return 1;
    }

    g_signal_connect(ros_sink, "new-sample",
                     G_CALLBACK(on_new_ros_image_sample),
                     publisher_context.get());
    gst_object_unref(ros_sink);
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

  ControlWindow window(overlay_state, pipeline);

  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  RCLCPP_INFO(node->get_logger(), "Stereo display pipeline started");

  if (app_cfg.has_unixfd_socket_path) {
    const std::string unixfd_socket_path = resolve_unixfd_socket_path(app_cfg);
    if (std::filesystem::exists(unixfd_socket_path)) {
      RCLCPP_INFO(node->get_logger(),
                  "Removing stale unixfd socket: %s",
                  unixfd_socket_path.c_str());
      std::filesystem::remove(unixfd_socket_path);
    }
  }

  g_app->run(window);

  RCLCPP_INFO(node->get_logger(), "Stereo display pipeline on quit: %s",
              pipeline_string.c_str());

  gst_element_send_event(pipeline, gst_event_new_eos());
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);

  g_app.reset();

  g_timestamp_pub.reset();

  rclcpp::shutdown();
  return 0;
}

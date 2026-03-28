#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <glib-unix.h>
#include <gst/video/video.h>

#include <filesystem>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <pwd.h>
#include <unistd.h>

#include "config.hpp"
#include "overlay.hpp"

namespace {

struct CommandLineOptions {
    std::vector<std::string> config_files;
};

struct CropValues {
    int left = 0;
    int right = 0;
    int top = 0;
    int bottom = 0;
};

enum class RosImageTarget {
    Left,
    Right,
    Stereo
};

struct RosImagePublisherContext {
    RosImageTarget target = RosImageTarget::Stereo;
    std::string topic_base;
    std::string frame_id;
    image_transport::CameraPublisher publisher;
};

static GMainLoop* g_main_loop = nullptr;

int clip_int(const int value, const int min_value, const int max_value) {
    return std::max(min_value, std::min(max_value, value));
}

std::pair<int, int> offset_valid_range(const int working_size, const int eye_size) {
    const int crop_total = std::max(0, working_size - eye_size);
    const int center = crop_total / 2;

    const int min_half = -center;
    const int max_half = crop_total - center;

    const int min_offset = static_cast<int>(std::ceil(2.0 * static_cast<double>(min_half))) + 1;
    const int max_offset = static_cast<int>(std::floor(2.0 * static_cast<double>(max_half))) - 1;

    if (min_offset > max_offset) {
        return {0, 0};
    }
    return {min_offset, max_offset};
}

int clamp_offset_to_valid(const int working_size, const int eye_size, const int offset_px) {
    const auto [min_offset, max_offset] = offset_valid_range(working_size, eye_size);
    return clip_int(offset_px, min_offset, max_offset);
}

std::pair<int, int> compute_axis_starts(const int crop_total, const int offset_px) {
    const int center = crop_total / 2;
    const int negative_start = center - static_cast<int>(std::floor(static_cast<double>(offset_px) / 2.0));
    const int positive_start = center + static_cast<int>(std::ceil(static_cast<double>(offset_px) / 2.0));
    return {negative_start, positive_start};
}

CropValues compute_eye_crop(
    const int working_w,
    const int working_h,
    const int eye_w,
    const int eye_h,
    const int baseline_px,
    const int vertical_offset_px,
    const int sign
) {
    const int crop_x_total = std::max(0, working_w - eye_w);
    const int crop_y_total = std::max(0, working_h - eye_h);

    const auto [left_start, right_start] = compute_axis_starts(crop_x_total, baseline_px);
    const auto [top_start, bottom_start] = compute_axis_starts(crop_y_total, vertical_offset_px);

    CropValues crop;
    crop.left = sign < 0 ? left_start : right_start;
    crop.right = crop_x_total - crop.left;
    crop.top = sign < 0 ? top_start : bottom_start;
    crop.bottom = crop_y_total - crop.top;
    return crop;
}

void print_usage(const char* executable) {
    std::cerr << "Usage: " << executable << " -c <config.json> [-c <config.json> ...]" << std::endl;
}

bool parse_arguments(int argc, char* argv[], CommandLineOptions& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-c" && i + 1 < argc) {
            options.config_files.emplace_back(argv[++i]);
            continue;
        }

        std::cerr << "Error: unknown argument '" << arg << "'." << std::endl;
        return false;
    }

    if (options.config_files.empty()) {
        std::cerr << "Error: at least one config file is required." << std::endl;
        return false;
    }

    return true;
}

bool validate_pipeline(const std::string& stream, const rclcpp::Logger& logger, const std::string& name) {
    if (stream.empty()) {
        RCLCPP_WARN(logger, "Video entry '%s' does not define a GStreamer stream yet", name.c_str());
        return false;
    }

    const std::vector<std::string> candidates = {
        stream,
        stream + " ! fakesink"
    };

    for (const auto& candidate : candidates) {
        GError* error = nullptr;
        GstElement* pipeline = gst_parse_launch(candidate.c_str(), &error);
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
                 "Unable to parse GStreamer stream for '%s'; expected either a full pipeline or a source snippet",
                 name.c_str());
    return false;
}

std::string resolve_unixfd_socket_path(const sv::AppConfig& stereo) {
    if (!stereo.has_unixfd_socket_path) {
        return "";
    }

    if (!stereo.unixfd_socket_path.empty()) {
        return stereo.unixfd_socket_path;
    }

    const char* username = getenv("USER");
    if (!username) {
        struct passwd* pw = getpwuid(getuid());
        username = pw ? pw->pw_name : "unknown";
    }
    return "/tmp/dvrk_stereo_viewer_" + std::string(username) + ".sock";
}

bool check_element_available(const std::string& element_name) {
    GstElementFactory* factory = gst_element_factory_find(element_name.c_str());
    if (factory) {
        gst_object_unref(factory);
        return true;
    }
    return false;
}

std::string get_unixfd_upload_chain() {
    if (check_element_available("nvvidconv")) {
        return "videoconvert ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12";
    }

    return "videoconvert ! video/x-raw,format=I420";
}

std::string to_lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
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

bool parse_ros_image_target(const std::string& value, RosImageTarget& target) {
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

bool has_ros_target(const std::vector<RosImageTarget>& targets, const RosImageTarget target) {
    return std::find(targets.begin(), targets.end(), target) != targets.end();
}

bool parse_ros_image_publishers(
    const std::vector<std::string>& values,
    const rclcpp::Logger& logger,
    std::vector<RosImageTarget>& targets
) {
    targets.clear();
    for (const auto& value : values) {
        RosImageTarget target;
        if (!parse_ros_image_target(value, target)) {
            RCLCPP_ERROR(logger,
                         "Invalid ros_image_publishers entry '%s'. Allowed values are: left, right, stereo",
                         value.c_str());
            return false;
        }
        if (!has_ros_target(targets, target)) {
            targets.push_back(target);
        }
    }
    return true;
}

GstFlowReturn on_new_ros_image_sample(GstElement* sink, gpointer user_data) {
    auto* publisher_context = static_cast<RosImagePublisherContext*>(user_data);
    if (publisher_context == nullptr) {
        return GST_FLOW_OK;
    }

    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (sample == nullptr) {
        return GST_FLOW_OK;
    }

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    GstVideoInfo video_info;

    if (buffer != nullptr && caps != nullptr && gst_video_info_from_caps(&video_info, caps)) {
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

            const uint8_t* src = static_cast<uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
            const int src_stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
            uint8_t* dst = image_msg->data.data();

            if (static_cast<int>(image_msg->step) == src_stride) {
                std::memcpy(dst, src, image_size);
            } else {
                for (uint32_t row = 0; row < image_msg->height; ++row) {
                    std::memcpy(dst + row * image_msg->step, src + row * src_stride, image_msg->step);
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

gboolean on_sigint(gpointer) {
    if (g_main_loop != nullptr) {
        g_main_loop_quit(g_main_loop);
    }
    return G_SOURCE_CONTINUE;
}

gboolean on_ros_spin(gpointer user_data) {
    if (user_data == nullptr || !rclcpp::ok()) {
        return G_SOURCE_CONTINUE;
    }

    auto* node = static_cast<rclcpp::Node*>(user_data);
    rclcpp::spin_some(node->get_node_base_interface());
    return G_SOURCE_CONTINUE;
}

gboolean on_bus_message(GstBus*, GstMessage* msg, gpointer) {
    if (msg == nullptr) {
        return G_SOURCE_CONTINUE;
    }

    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
        if (g_main_loop != nullptr) {
            g_main_loop_quit(g_main_loop);
        }
    } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
        GError* err = nullptr;
        gchar* dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        std::cerr << "GStreamer error: " << (err ? err->message : "unknown") << std::endl;
        if (dbg != nullptr) {
            std::cerr << "Debug details: " << dbg << std::endl;
            g_free(dbg);
        }
        if (err != nullptr) {
            g_error_free(err);
        }
        if (g_main_loop != nullptr) {
            g_main_loop_quit(g_main_loop);
        }
    }

    return G_SOURCE_CONTINUE;
}

std::string build_pipeline_string(
    const sv::AppConfig& stereo,
    const std::vector<RosImageTarget>& ros_targets,
    const bool include_overlay
) {
    const int eye_w = stereo.crop_width;
    const int eye_h = stereo.crop_height;

    const bool publish_left = has_ros_target(ros_targets, RosImageTarget::Left);
    const bool publish_right = has_ros_target(ros_targets, RosImageTarget::Right);
    const bool publish_stereo = has_ros_target(ros_targets, RosImageTarget::Stereo);

    int horizontal_shift_px = clamp_offset_to_valid(stereo.original_width, eye_w, stereo.horizontal_shift_px);
    int vertical_shift_px = clamp_offset_to_valid(stereo.original_height, eye_h, stereo.vertical_shift_px);

    const CropValues left_crop = compute_eye_crop(
        stereo.original_width,
        stereo.original_height,
        eye_w,
        eye_h,
        horizontal_shift_px,
        vertical_shift_px,
        -1
    );

    const CropValues right_crop = compute_eye_crop(
        stereo.original_width,
        stereo.original_height,
        eye_w,
        eye_h,
        horizontal_shift_px,
        vertical_shift_px,
        1
    );

    std::string left_chain =
        stereo.left.source +
        " ! queue max-size-buffers=1 leaky=downstream"
        " ! videoconvert"
        " ! videocrop left=" + std::to_string(left_crop.left) +
        " right=" + std::to_string(left_crop.right) +
        " top=" + std::to_string(left_crop.top) +
        " bottom=" + std::to_string(left_crop.bottom);
    if (publish_left) {
        left_chain +=
            " ! tee name=__left_out__"
            " __left_out__. ! queue max-size-buffers=1 leaky=downstream ! mix.sink_0"
            " __left_out__. ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream"
            " ! videoconvert ! video/x-raw,format=RGB"
            " ! appsink name=__ros_left_sink__ sync=false async=false emit-signals=true drop=true max-buffers=1";
    } else {
        left_chain += " ! mix.sink_0";
    }

    std::string right_chain =
        stereo.right.source +
        " ! queue max-size-buffers=1 leaky=downstream"
        " ! videoconvert"
        " ! videocrop left=" + std::to_string(right_crop.left) +
        " right=" + std::to_string(right_crop.right) +
        " top=" + std::to_string(right_crop.top) +
        " bottom=" + std::to_string(right_crop.bottom);
    if (publish_right) {
        right_chain +=
            " ! tee name=__right_out__"
            " __right_out__. ! queue max-size-buffers=1 leaky=downstream ! mix.sink_1"
            " __right_out__. ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream"
            " ! videoconvert ! video/x-raw,format=RGB"
            " ! appsink name=__ros_right_sink__ sync=false async=false emit-signals=true drop=true max-buffers=1";
    } else {
        right_chain += " ! mix.sink_1";
    }

    std::string output_chain =
        "compositor name=mix sink_0::xpos=0 sink_1::xpos=" + std::to_string(eye_w) +
        " ! video/x-raw,width=" + std::to_string(2 * eye_w) +
        ",height=" + std::to_string(eye_h);

    const bool need_stereo_tee = stereo.has_unixfd_socket_path || publish_stereo;
    if (need_stereo_tee) {
        output_chain += " ! tee name=__stereo_out__ ";
        output_chain += "__stereo_out__. ! queue max-size-buffers=1 leaky=downstream";
        if (include_overlay) {
            output_chain += " ! cairooverlay name=stereo_overlay";
        }
        output_chain += " ! videoconvert ! " + stereo.sink;

        if (stereo.has_unixfd_socket_path) {
            const std::string socket_path = resolve_unixfd_socket_path(stereo);
            const std::string unixfd_upload_chain = get_unixfd_upload_chain();
            output_chain += " __stereo_out__. ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream"
                            " ! " + unixfd_upload_chain +
                            " ! unixfdsink socket-path=" + socket_path + " sync=false async=false";
        }

        if (publish_stereo) {
            output_chain += " __stereo_out__. ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream"
                            " ! videoconvert ! video/x-raw,format=RGB"
                            " ! appsink name=__ros_stereo_sink__ sync=false async=false emit-signals=true drop=true max-buffers=1";
        }
    } else {
        if (include_overlay) {
            output_chain += " ! cairooverlay name=stereo_overlay";
        }
        output_chain += " ! videoconvert ! " + stereo.sink;
    }

    return left_chain + " " + right_chain + " " + output_chain;
}

}  // namespace

int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);
    rclcpp::init(argc, argv);

    CommandLineOptions options;
    if (!parse_arguments(argc, argv, options)) {
        print_usage(argv[0]);
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<rclcpp::Node>("dvrk_stereo_viewer");
    auto overlay_state = std::make_shared<sv::OverlayState>();

    std::string console_name = "console";

    const bool overlay_available = check_element_available("cairooverlay");
    if (!overlay_available) {
        RCLCPP_WARN(node->get_logger(), "GStreamer element 'cairooverlay' is unavailable; dVRK status overlay is disabled");
    }

    std::vector<sv::AppConfig> app_configs;
    for (const auto& path : options.config_files) {
        if (!std::filesystem::exists(path)) {
            RCLCPP_ERROR(node->get_logger(), "Config file does not exist: %s", path.c_str());
            rclcpp::shutdown();
            return 1;
        }

        Json::Value root;
        if (!sv::Config::load_from_file(path, root)) {
            rclcpp::shutdown();
            return 1;
        }

        if (!sv::Config::check_type(root, "dc::stereo_viewer_config@1.0.0", path)) {
            rclcpp::shutdown();
            return 1;
        }

        app_configs.push_back(sv::Config::parse_app_config(root));
    }

    std::string pipeline_string;
    std::string selected_viewer_name = "dvrk_stereo_viewer";
    std::vector<RosImageTarget> selected_ros_targets;
    for (const auto& cfg : app_configs) {
        RCLCPP_INFO(node->get_logger(), "Loaded viewer config: %s", cfg.name.c_str());
        console_name = cfg.dvrk_console_namespace;
        overlay_state->overlay_alpha = cfg.overlay_alpha;
        selected_viewer_name = cfg.name.empty() ? "dvrk_stereo_viewer" : cfg.name;
        const sv::AppConfig& app_cfg = cfg;

        if (!parse_ros_image_publishers(cfg.ros_image_publishers, node->get_logger(), selected_ros_targets)) {
            rclcpp::shutdown();
            return 1;
        }

        if (!selected_ros_targets.empty()) {
            std::string topics;
            const std::string topic_prefix = trim_topic_tokens(selected_viewer_name);
            for (const auto& target : selected_ros_targets) {
                if (!topics.empty()) {
                    topics += ", ";
                }
                topics += topic_prefix + "/" + ros_image_target_name(target) + "/image_raw";
            }
            RCLCPP_INFO(node->get_logger(), "ROS image publishing enabled for: %s", topics.c_str());
        }

        if (app_cfg.left.source.empty() || app_cfg.right.source.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Config '%s' must define left_stream and right_stream", cfg.name.c_str());
            rclcpp::shutdown();
            return 1;
        }

        if (app_cfg.crop_width <= 0 || app_cfg.crop_height <= 0 ||
            app_cfg.original_width <= 0 || app_cfg.original_height <= 0) {
            RCLCPP_ERROR(node->get_logger(),
                         "Config '%s' must provide positive original_width/original_height and crop_width/crop_height",
                         cfg.name.c_str());
            rclcpp::shutdown();
            return 1;
        }

        const std::string unixfd_upload_chain = get_unixfd_upload_chain();
        const std::string unixfd_socket_path = resolve_unixfd_socket_path(app_cfg);
        if (app_cfg.has_unixfd_socket_path) {
            if (app_cfg.unixfd_socket_path.empty()) {
                RCLCPP_INFO(node->get_logger(), "unixfd publish path (default): %s", unixfd_socket_path.c_str());
            } else {
                RCLCPP_INFO(node->get_logger(), "unixfd publish path: %s", unixfd_socket_path.c_str());
            }
            RCLCPP_INFO(node->get_logger(), "unixfd export chain: %s", unixfd_upload_chain.c_str());
        } else {
            RCLCPP_INFO(node->get_logger(), "unixfd publish disabled (unixfd_socket_path is set to empty)");
        }

        pipeline_string = build_pipeline_string(app_cfg, selected_ros_targets, overlay_available);
        break;
    }

    std::vector<std::unique_ptr<RosImagePublisherContext>> ros_publisher_contexts;
    std::shared_ptr<image_transport::ImageTransport> ros_image_transport;
    if (!selected_ros_targets.empty()) {
        ros_image_transport = std::make_shared<image_transport::ImageTransport>(node);
        const std::string viewer_topic_base = trim_topic_tokens(selected_viewer_name);
        for (const auto& target : selected_ros_targets) {
            auto publisher_context = std::make_unique<RosImagePublisherContext>();
            publisher_context->target = target;
            publisher_context->topic_base = viewer_topic_base + "/" + ros_image_target_name(target);
            publisher_context->frame_id = publisher_context->topic_base + "_frame";
            publisher_context->publisher = ros_image_transport->advertiseCamera(publisher_context->topic_base + "/image_raw", 10);
            ros_publisher_contexts.push_back(std::move(publisher_context));
        }
    }

    const std::string camera_topic = "/" + console_name + "/camera";
    const std::string clutch_topic = "/" + console_name + "/clutch";
    const std::string teleop_selected_topic = "/" + console_name + "/teleop/selected";
    const std::string teleop_unselected_topic = "/" + console_name + "/teleop/unselected";
    RCLCPP_INFO(node->get_logger(),
                "Console topics: camera=%s clutch=%s teleop_selected=%s teleop_unselected=%s",
                camera_topic.c_str(),
                clutch_topic.c_str(),
                teleop_selected_topic.c_str(),
                teleop_unselected_topic.c_str());

    const auto latch_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto teleop_selected_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().transient_local();
    auto camera_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        camera_topic,
        latch_qos,
        [overlay_state](const sensor_msgs::msg::Joy::SharedPtr msg) {
            sv::on_camera_joy(msg, overlay_state);
        }
    );

    auto clutch_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        clutch_topic,
        latch_qos,
        [overlay_state](const sensor_msgs::msg::Joy::SharedPtr msg) {
            sv::on_clutch_joy(msg, overlay_state);
        }
    );

    auto following_subscribers_cache = std::make_shared<std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>>();
    auto active_teleops = std::make_shared<std::unordered_set<std::string>>();
    auto latest_teleop_by_mtm = std::make_shared<std::unordered_map<std::string, std::string>>();
    auto teleop_selected_sub = node->create_subscription<std_msgs::msg::String>(
        teleop_selected_topic,
        teleop_selected_qos,
        [node, overlay_state, following_subscribers_cache, active_teleops, latest_teleop_by_mtm, latch_qos](const std_msgs::msg::String::SharedPtr msg) {
            if (msg == nullptr) {
                return;
            }

            std::string mtm_name;
            sv::TeleopSide side;
            int psm_number = 0;
            if (!sv::parse_teleop_name(msg->data, mtm_name, side, psm_number)) {
                return;
            }

            const std::string teleop_name = msg->data;
            const auto latest_it = latest_teleop_by_mtm->find(mtm_name);
            if (latest_it != latest_teleop_by_mtm->end() && latest_it->second != teleop_name) {
                const std::string previous_teleop = latest_it->second;
                active_teleops->erase(previous_teleop);
                auto previous_msg = std::make_shared<std_msgs::msg::String>();
                previous_msg->data = previous_teleop;
                sv::on_teleop_unselected(previous_msg, overlay_state);
                RCLCPP_INFO(node->get_logger(),
                            "Replacing teleop for %s: %s -> %s",
                            mtm_name.c_str(),
                            previous_teleop.c_str(),
                            teleop_name.c_str());
            }
            (*latest_teleop_by_mtm)[mtm_name] = teleop_name;
            active_teleops->insert(teleop_name);

            sv::on_teleop_selected(msg, overlay_state);

            if (following_subscribers_cache->find(teleop_name) != following_subscribers_cache->end()) {
                return;
            }
            const std::string following_topic = "/" + teleop_name + "/following";

            auto following_sub = node->create_subscription<std_msgs::msg::Bool>(
                following_topic,
                latch_qos,
                [overlay_state, active_teleops, teleop_name](const std_msgs::msg::Bool::SharedPtr following_msg) {
                    if (active_teleops->find(teleop_name) == active_teleops->end()) {
                        return;
                    }
                    sv::on_teleop_following(teleop_name, following_msg, overlay_state);
                }
            );

            (*following_subscribers_cache)[teleop_name] = following_sub;
            RCLCPP_INFO(node->get_logger(), "Cached teleop following subscriber: %s", following_topic.c_str());
        }
    );

    auto teleop_unselected_sub = node->create_subscription<std_msgs::msg::String>(
        teleop_unselected_topic,
        latch_qos,
        [node, overlay_state, active_teleops, latest_teleop_by_mtm](const std_msgs::msg::String::SharedPtr msg) {
            if (msg == nullptr) {
                return;
            }

            std::string mtm_name;
            sv::TeleopSide side;
            int psm_number = 0;
            const bool parsed = sv::parse_teleop_name(msg->data, mtm_name, side, psm_number);

            sv::on_teleop_unselected(msg, overlay_state);
            active_teleops->erase(msg->data);
            RCLCPP_INFO(node->get_logger(), "Teleop inactive (subscriber cached): /%s/following", msg->data.c_str());

            if (parsed) {
                const auto latest_it = latest_teleop_by_mtm->find(mtm_name);
                if (latest_it != latest_teleop_by_mtm->end() && latest_it->second == msg->data) {
                    latest_teleop_by_mtm->erase(latest_it);
                }
            }
        }
    );

    if (!validate_pipeline(pipeline_string, node->get_logger(), "dvrk_stereo_viewer_pipeline")) {
        rclcpp::shutdown();
        return 1;
    }

    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(pipeline_string.c_str(), &error);
    if (error != nullptr || pipeline == nullptr) {
        if (error != nullptr) {
            RCLCPP_ERROR(node->get_logger(), "Failed to create pipeline: %s", error->message);
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

    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    gst_bus_add_watch(bus, on_bus_message, nullptr);
    gst_object_unref(bus);

    if (overlay_available) {
        GstElement* overlay = gst_bin_get_by_name(GST_BIN(pipeline), "stereo_overlay");
        if (overlay != nullptr) {
            g_signal_connect(overlay, "caps-changed", G_CALLBACK(sv::on_overlay_caps_changed), overlay_state.get());
            g_signal_connect(overlay, "draw", G_CALLBACK(sv::on_overlay_draw), overlay_state.get());
            gst_object_unref(overlay);
        } else {
            RCLCPP_WARN(node->get_logger(), "Unable to find 'stereo_overlay' element in pipeline; dVRK status overlay is disabled");
        }
    }

    for (auto& publisher_context : ros_publisher_contexts) {
        GstElement* ros_sink = gst_bin_get_by_name(
            GST_BIN(pipeline),
            ros_sink_name(publisher_context->target).c_str()
        );
        if (ros_sink == nullptr) {
            RCLCPP_ERROR(node->get_logger(),
                         "Failed to find GStreamer sink '%s' for ROS image target '%s'",
                         ros_sink_name(publisher_context->target).c_str(),
                         ros_image_target_name(publisher_context->target).c_str());
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
            rclcpp::shutdown();
            return 1;
        }

        g_signal_connect(ros_sink, "new-sample", G_CALLBACK(on_new_ros_image_sample), publisher_context.get());
        gst_object_unref(ros_sink);
    }

    g_main_loop = g_main_loop_new(nullptr, FALSE);
    g_unix_signal_add(SIGINT, on_sigint, nullptr);
    g_unix_signal_add(SIGTERM, on_sigint, nullptr);
    g_timeout_add(20, on_ros_spin, node.get());

    rclcpp::spin_some(node->get_node_base_interface());

    (void)camera_sub;
    (void)clutch_sub;
    (void)teleop_selected_sub;
    (void)teleop_unselected_sub;

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    RCLCPP_INFO(node->get_logger(), "Stereo viewer pipeline started");
    g_main_loop_run(g_main_loop);

    RCLCPP_INFO(node->get_logger(), "Stereo viewer pipeline on quit: %s", pipeline_string.c_str());

    gst_element_send_event(pipeline, gst_event_new_eos());
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    g_main_loop_unref(g_main_loop);
    g_main_loop = nullptr;

    rclcpp::shutdown();
    return 0;
}

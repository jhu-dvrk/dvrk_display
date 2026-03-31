#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <rclcpp/rclcpp.hpp>
#include <glib-unix.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "config.hpp"
#include "overlay.hpp"
#include "gst_utils.hpp"
#include "ros_utils.hpp"

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
    if (min_offset > max_offset) return {0, 0};
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

CropValues compute_eye_crop(const int working_w, const int working_h, const int eye_w, const int eye_h, const int baseline_px, const int vertical_offset_px, const int sign) {
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
    std::cerr << "Usage: " << executable << " -c <config.json>" << std::endl;
}

bool parse_arguments(int argc, char* argv[], CommandLineOptions& options) {
    bool seen_config = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-c" && i + 1 < argc) {
            options.config_file = argv[++i];
            seen_config = true;
            continue;
        }
        std::cerr << "Error: unknown argument '" << arg << "'." << std::endl;
        return false;
    }
    if (!seen_config) {
        std::cerr << "Error: config file is required." << std::endl;
        return false;
    }
    return true;
}

gboolean on_sigint_cb(gpointer) {
    if (g_main_loop != nullptr) g_main_loop_quit(g_main_loop);
    return G_SOURCE_CONTINUE;
}

gboolean on_ros_spin_cb(gpointer user_data) {
    if (user_data == nullptr || !rclcpp::ok()) return G_SOURCE_CONTINUE;
    auto* node = static_cast<rclcpp::Node*>(user_data);
    rclcpp::spin_some(node->get_node_base_interface());
    return G_SOURCE_CONTINUE;
}

std::string build_pipeline_string(const sv::AppConfig& stereo, const rclcpp::Logger& logger) {
    const int eye_w = stereo.crop_width;
    const int eye_h = stereo.crop_height;
    const int working_w = stereo.original_width;
    const int working_h = stereo.original_height;
    const int baseline_px = clamp_offset_to_valid(working_w, eye_w, stereo.horizontal_shift_px);
    const int vertical_offset_px = clamp_offset_to_valid(working_h, eye_h, stereo.vertical_shift_px);
    const CropValues left_crop = compute_eye_crop(working_w, working_h, eye_w, eye_h, baseline_px, vertical_offset_px, -1);
    const CropValues right_crop = compute_eye_crop(working_w, working_h, eye_w, eye_h, baseline_px, vertical_offset_px, 1);

    std::vector<sv::RosImageTarget> ros_targets;
    sv::parse_ros_image_publishers(stereo.ros_image_publishers, logger, ros_targets);

    const bool publish_left = sv::has_ros_target(ros_targets, sv::RosImageTarget::Left);
    const bool publish_right = sv::has_ros_target(ros_targets, sv::RosImageTarget::Right);
    const bool publish_stereo = sv::has_ros_target(ros_targets, sv::RosImageTarget::Stereo);
    const bool include_overlay = sv::check_element_available("cairooverlay");

    std::string left_chain = stereo.left.source + " ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! videocrop left=" + std::to_string(left_crop.left) + " right=" + std::to_string(left_crop.right) + " top=" + std::to_string(left_crop.top) + " bottom=" + std::to_string(left_crop.bottom);
    if (publish_left) {
        left_chain += " ! tee name=__left_out__ __left_out__. ! queue max-size-buffers=1 leaky=downstream ! mix.sink_0 __left_out__. ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! video/x-raw,format=RGB ! appsink name=__ros_left_sink__ sync=false async=false emit-signals=true drop=true max-buffers=1";
    } else {
        left_chain += " ! mix.sink_0";
    }

    std::string right_chain = stereo.right.source + " ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! videocrop left=" + std::to_string(right_crop.left) + " right=" + std::to_string(right_crop.right) + " top=" + std::to_string(right_crop.top) + " bottom=" + std::to_string(right_crop.bottom);
    if (publish_right) {
        right_chain += " ! tee name=__right_out__ __right_out__. ! queue max-size-buffers=1 leaky=downstream ! mix.sink_1 __right_out__. ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! video/x-raw,format=RGB ! appsink name=__ros_right_sink__ sync=false async=false emit-signals=true drop=true max-buffers=1";
    } else {
        right_chain += " ! mix.sink_1";
    }

    std::string output_chain = "compositor name=mix sink_0::xpos=0 sink_1::xpos=" + std::to_string(eye_w) + " ! video/x-raw,width=" + std::to_string(2 * eye_w) + ",height=" + std::to_string(eye_h);
    const bool need_stereo_tee = stereo.has_unixfd_socket_path || publish_stereo;
    if (need_stereo_tee) {
        output_chain += " ! tee name=__stereo_out__ __stereo_out__. ! queue max-size-buffers=1 leaky=downstream";
        if (include_overlay) output_chain += " ! cairooverlay name=stereo_overlay";
        output_chain += " ! videoconvert ! " + stereo.sink_streams[0];
        if (stereo.has_unixfd_socket_path) {
            output_chain += " __stereo_out__. ! queue max-size-buffers=2 leaky=downstream ! " + sv::get_unixfd_upload_chain() + " ! unixfdsink socket-path=" + sv::resolve_unixfd_socket_path(stereo) + " sync=false async=false";
        }
        if (publish_stereo) {
            output_chain += " __stereo_out__. ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! video/x-raw,format=RGB ! appsink name=__ros_stereo_sink__ sync=false async=false emit-signals=true drop=true max-buffers=1";
        }
    } else {
        if (include_overlay) output_chain += " ! cairooverlay name=stereo_overlay";
        output_chain += " ! videoconvert ! " + stereo.sink_streams[0];
    }
    return left_chain + " " + right_chain + " " + output_chain;
}

} // namespace

int main(int argc, char* argv[]) {
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

    const bool overlay_available = sv::check_element_available("cairooverlay");
    const std::string& path = options.config_file;
    Json::Value root;
    if (!sv::Config::load_from_file(path, root) || !sv::Config::check_type(root, "sv::dvrk_display_config@1.0.0", path)) {
        rclcpp::shutdown();
        return 1;
    }

    const sv::AppConfig cfg = sv::Config::parse_app_config(root);
    overlay_state->overlay_alpha = cfg.overlay_alpha;
    sv::warn_if_interlaced_stream(cfg.left.source, node->get_logger(), "left");
    sv::warn_if_interlaced_stream(cfg.right.source, node->get_logger(), "right");

    std::string pipeline_string = build_pipeline_string(cfg, node->get_logger());
    if (!sv::validate_pipeline(pipeline_string, node->get_logger(), "stereo_display")) {
        rclcpp::shutdown();
        return 1;
    }

    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(pipeline_string.c_str(), &error);
    if (error || !pipeline) {
        if (error) {
            std::cerr << "Pipeline error: " << error->message << std::endl;
            g_error_free(error);
        }
        rclcpp::shutdown();
        return 1;
    }

    sv::setup_dvrk_overlay_subscriptions(node, overlay_state, cfg.dvrk_console_namespace);

    g_main_loop = g_main_loop_new(nullptr, FALSE);
    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    gst_bus_add_watch(bus, sv::on_bus_message, g_main_loop);
    gst_object_unref(bus);

    if (overlay_available) {
        GstElement* overlay = gst_bin_get_by_name(GST_BIN(pipeline), "stereo_overlay");
        if (overlay) {
            g_signal_connect(overlay, "caps-changed", G_CALLBACK(sv::on_overlay_caps_changed), overlay_state.get());
            g_signal_connect(overlay, "draw", G_CALLBACK(sv::on_overlay_draw), overlay_state.get());
            gst_object_unref(overlay);
        }
    }

    std::vector<sv::RosImageTarget> targets;
    sv::parse_ros_image_publishers(cfg.ros_image_publishers, node->get_logger(), targets);
    auto it = std::make_shared<image_transport::ImageTransport>(node);
    static std::vector<std::unique_ptr<sv::RosImagePublisherContext>> static_ros_contexts;
    static_ros_contexts.clear();

    for (auto target : targets) {
        GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), sv::ros_sink_name(target).c_str());
        if (sink) {
            auto ctx = std::make_unique<sv::RosImagePublisherContext>();
            ctx->target = target;
            ctx->topic_base = sv::trim_topic_tokens(cfg.name) + "/" + sv::ros_image_target_name(target);
            ctx->frame_id = ctx->topic_base + "_frame";
            ctx->publisher = it->advertiseCamera(ctx->topic_base + "/image_raw", 10);
            
            g_signal_connect(sink, "new-sample", G_CALLBACK(sv::on_new_ros_image_sample), ctx.get());
            static_ros_contexts.push_back(std::move(ctx));
            gst_object_unref(sink);
        }
    }

    g_unix_signal_add(SIGINT, on_sigint_cb, nullptr);
    g_unix_signal_add(SIGTERM, on_sigint_cb, nullptr);
    g_timeout_add(20, on_ros_spin_cb, node.get());

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    g_main_loop_run(g_main_loop);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    
    // Clear the static contexts to release ROS resources before shutdown
    static_ros_contexts.clear();
    sv::cleanup_dvrk_overlay_subscriptions();
    
    g_main_loop_unref(g_main_loop);
    rclcpp::shutdown();
    return 0;
}

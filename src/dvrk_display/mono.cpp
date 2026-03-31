#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <rclcpp/rclcpp.hpp>
#include <glib-unix.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "config.hpp"
#include "overlay.hpp"
#include "gst_utils.hpp"
#include "ros_utils.hpp"

namespace {

struct CommandLineOptions {
    std::string config_file;
};

static GMainLoop* g_main_loop = nullptr;

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

gboolean on_sigint(gpointer) {
    if (g_main_loop != nullptr) g_main_loop_quit(g_main_loop);
    return G_SOURCE_CONTINUE;
}

gboolean on_ros_spin_cb(gpointer user_data) {
    if (user_data == nullptr || !rclcpp::ok()) return G_SOURCE_CONTINUE;
    auto* node = static_cast<rclcpp::Node*>(user_data);
    rclcpp::spin_some(node->get_node_base_interface());
    return G_SOURCE_CONTINUE;
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

    auto node = std::make_shared<rclcpp::Node>("dvrk_mono_display");
    auto overlay_state = std::make_shared<sv::OverlayState>();
    overlay_state->is_stereo = false;

    const bool overlay_available = sv::check_element_available("cairooverlay");
    if (!overlay_available) {
        RCLCPP_WARN(node->get_logger(), "GStreamer element 'cairooverlay' is unavailable; dVRK status overlay is disabled");
    }

    const std::string& path = options.config_file;
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

    if (!sv::Config::check_type(root, "sv::mono_display_config@1.0.0", path)) {
        rclcpp::shutdown();
        return 1;
    }

    const sv::AppConfig cfg = sv::Config::parse_app_config(root);
    overlay_state->overlay_alpha = cfg.overlay_alpha;

    std::vector<sv::RosImageTarget> ros_targets;
    if (!sv::parse_ros_image_publishers(cfg.ros_image_publishers, node->get_logger(), ros_targets)) {
        rclcpp::shutdown();
        return 1;
    }

    // Build pipeline
    // For mono, we just use cfg.left.source as the primary stream
    std::string pipeline_str = cfg.left.source + " ! queue max-size-buffers=1 leaky=downstream ! videoconvert";
    
    if (overlay_available) {
        pipeline_str += " ! cairooverlay name=mono_overlay";
    }

    if (sv::has_ros_target(ros_targets, sv::RosImageTarget::Mono)) {
        pipeline_str += " ! tee name=t t. ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! " + cfg.sink_streams[0];
        pipeline_str += " t. ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! video/x-raw,format=RGB ! appsink name=__ros_mono_sink__ sync=false async=false emit-signals=true drop=true max-buffers=1";
    } else {
        pipeline_str += " ! videoconvert ! " + cfg.sink_streams[0];
    }

    if (!sv::validate_pipeline(pipeline_str, node->get_logger(), "mono_display")) {
        rclcpp::shutdown();
        return 1;
    }

    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error || !pipeline) {
        RCLCPP_ERROR(node->get_logger(), "Failed to launch pipeline: %s", error ? error->message : "unknown");
        if (error) g_error_free(error);
        rclcpp::shutdown();
        return 1;
    }

    sv::setup_dvrk_overlay_subscriptions(node, overlay_state, cfg.dvrk_console_namespace);

    g_main_loop = g_main_loop_new(nullptr, FALSE);
    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    gst_bus_add_watch(bus, sv::on_bus_message, g_main_loop);
    gst_object_unref(bus);

    if (overlay_available) {
        GstElement* overlay = gst_bin_get_by_name(GST_BIN(pipeline), "mono_overlay");
        if (overlay) {
            g_signal_connect(overlay, "caps-changed", G_CALLBACK(sv::on_overlay_caps_changed), overlay_state.get());
            g_signal_connect(overlay, "draw", G_CALLBACK(sv::on_overlay_draw), overlay_state.get());
            gst_object_unref(overlay);
        }
    }

    static std::shared_ptr<sv::RosImagePublisherContext> static_ctx_storage;
    if (sv::has_ros_target(ros_targets, sv::RosImageTarget::Mono)) {
        GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "__ros_mono_sink__");
        if (sink) {
            auto it = std::make_shared<image_transport::ImageTransport>(node);
            auto ctx = std::make_unique<sv::RosImagePublisherContext>();
            ctx->target = sv::RosImageTarget::Mono;
            ctx->topic_base = sv::trim_topic_tokens(cfg.name) + "/mono";
            ctx->frame_id = ctx->topic_base + "_frame";
            ctx->publisher = it->advertiseCamera(ctx->topic_base + "/image_raw", 10);
            
            static_ctx_storage = std::move(ctx);
            g_signal_connect(sink, "new-sample", G_CALLBACK(sv::on_new_ros_image_sample), static_ctx_storage.get());
            gst_object_unref(sink);
        }
    }

    g_unix_signal_add(SIGINT, on_sigint, nullptr);
    g_unix_signal_add(SIGTERM, on_sigint, nullptr);
    g_timeout_add(20, on_ros_spin_cb, node.get());

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    g_main_loop_run(g_main_loop);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    
    // Clear the static context to release ROS resources before shutdown
    static_ctx_storage.reset();
    sv::cleanup_dvrk_overlay_subscriptions();
    
    g_main_loop_unref(g_main_loop);
    rclcpp::shutdown();
    return 0;
}

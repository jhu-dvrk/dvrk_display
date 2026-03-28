#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <glib-unix.h>

#include <cairo/cairo.h>
#include <gst/video/video.h>

#include <filesystem>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <pwd.h>
#include <unistd.h>

#include "config.hpp"

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

struct OverlayState {
    bool has_camera = false;
    bool has_clutch = false;
    bool camera_active = false;
    bool clutch_active = false;
    int frame_width = 0;
    int frame_height = 0;
    std::mutex mutex;
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

std::string resolve_unixfd_socket_path(const sv::StereoConfig& stereo) {
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

gboolean on_sigint(gpointer) {
    if (g_main_loop != nullptr) {
        g_main_loop_quit(g_main_loop);
    }
    return G_SOURCE_CONTINUE;
}

bool joy_active(const sensor_msgs::msg::Joy& msg) {
    const bool any_button_pressed = std::any_of(
        msg.buttons.begin(), msg.buttons.end(),
        [](const int value) {
            return value != 0;
        }
    );
    if (any_button_pressed) {
        return true;
    }

    return std::any_of(
        msg.axes.begin(), msg.axes.end(),
        [](const float value) {
            return std::abs(value) > 0.5F;
        }
    );
}

void on_camera_joy(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    overlay_state->has_camera = true;
    overlay_state->camera_active = joy_active(*msg);
}

void on_clutch_joy(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::shared_ptr<OverlayState>& overlay_state
) {
    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    overlay_state->has_clutch = true;
    overlay_state->clutch_active = joy_active(*msg);
}

void on_overlay_caps_changed(GstElement*, GstCaps* caps, gpointer user_data) {
    if (caps == nullptr || user_data == nullptr) {
        return;
    }

    GstVideoInfo video_info;
    if (!gst_video_info_from_caps(&video_info, caps)) {
        return;
    }

    auto* overlay_state = static_cast<OverlayState*>(user_data);
    std::scoped_lock<std::mutex> lock(overlay_state->mutex);
    overlay_state->frame_width = static_cast<int>(video_info.width);
    overlay_state->frame_height = static_cast<int>(video_info.height);
}

void draw_status_block(cairo_t* cr, const std::string& line_1, const std::string& line_2, const double x, const double y) {
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 0.55);
    cairo_rectangle(cr, x - 12.0, y - 28.0, 280.0, 58.0);
    cairo_fill(cr);

    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 20.0);

    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);
    cairo_move_to(cr, x, y);
    cairo_show_text(cr, line_1.c_str());

    cairo_set_source_rgba(cr, 0.95, 0.95, 0.95, 1.0);
    cairo_move_to(cr, x, y + 24.0);
    cairo_show_text(cr, line_2.c_str());
}

void on_overlay_draw(GstElement*, cairo_t* cr, guint64, guint64, gpointer user_data) {
    if (cr == nullptr || user_data == nullptr) {
        return;
    }

    auto* overlay_state = static_cast<OverlayState*>(user_data);
    bool has_camera = false;
    bool has_clutch = false;
    bool camera_active = false;
    bool clutch_active = false;
    int frame_width = 0;

    {
        std::scoped_lock<std::mutex> lock(overlay_state->mutex);
        has_camera = overlay_state->has_camera;
        has_clutch = overlay_state->has_clutch;
        camera_active = overlay_state->camera_active;
        clutch_active = overlay_state->clutch_active;
        frame_width = overlay_state->frame_width;
    }

    const std::string camera_text = "CAMERA: " + std::string(has_camera ? (camera_active ? "ON" : "OFF") : "--");
    const std::string clutch_text = "CLUTCH: " + std::string(has_clutch ? (clutch_active ? "ON" : "OFF") : "--");

    const double left_x = 20.0;
    const double left_y = 36.0;
    draw_status_block(cr, camera_text, clutch_text, left_x, left_y);

    if (frame_width > 0) {
        const double right_x = static_cast<double>(frame_width / 2 + 20);
        draw_status_block(cr, camera_text, clutch_text, right_x, left_y);
    }
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

std::string build_pipeline_string(const sv::StereoConfig& stereo, const bool include_overlay) {
    const int eye_w = stereo.crop_width;
    const int eye_h = stereo.crop_height;

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

    const std::string left_chain =
        stereo.left.source +
        " ! queue max-size-buffers=1 leaky=downstream"
        " ! videoconvert"
        " ! videocrop left=" + std::to_string(left_crop.left) +
        " right=" + std::to_string(left_crop.right) +
        " top=" + std::to_string(left_crop.top) +
        " bottom=" + std::to_string(left_crop.bottom) +
        " ! mix.sink_0";

    const std::string right_chain =
        stereo.right.source +
        " ! queue max-size-buffers=1 leaky=downstream"
        " ! videoconvert"
        " ! videocrop left=" + std::to_string(right_crop.left) +
        " right=" + std::to_string(right_crop.right) +
        " top=" + std::to_string(right_crop.top) +
        " bottom=" + std::to_string(right_crop.bottom) +
        " ! mix.sink_1";

    std::string output_chain =
        "compositor name=mix sink_0::xpos=0 sink_1::xpos=" + std::to_string(eye_w) +
        " ! video/x-raw,width=" + std::to_string(2 * eye_w) +
        ",height=" + std::to_string(eye_h);

    if (stereo.has_unixfd_socket_path) {
        const std::string socket_path = resolve_unixfd_socket_path(stereo);
        const std::string unixfd_upload_chain = get_unixfd_upload_chain();
        output_chain += " ! tee name=__stereo_out__ ";
        output_chain += "__stereo_out__. ! queue max-size-buffers=1 leaky=downstream";
        if (include_overlay) {
            output_chain += " ! cairooverlay name=stereo_overlay";
        }
        output_chain += " ! videoconvert ! " + stereo.sink + " ";
        output_chain += "__stereo_out__. ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! " + unixfd_upload_chain + " ! unixfdsink socket-path=" + socket_path + " sync=false async=false";
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
    auto overlay_state = std::make_shared<OverlayState>();

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

        if (!sv::Config::check_type(root, "stereo_viewer_config_v1", path)) {
            rclcpp::shutdown();
            return 1;
        }

        app_configs.push_back(sv::Config::parse_app_config(root));
    }

    std::string pipeline_string;
    for (const auto& cfg : app_configs) {
        RCLCPP_INFO(node->get_logger(), "Loaded viewer config: %s", cfg.viewer_name.c_str());
        console_name = cfg.console;
        sv::StereoConfig stereo_cfg = cfg.stereo;

        if (stereo_cfg.left.source.empty() || stereo_cfg.right.source.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Config '%s' must define stereo.left_stream and stereo.right_stream", cfg.viewer_name.c_str());
            rclcpp::shutdown();
            return 1;
        }

        if (stereo_cfg.crop_width <= 0 || stereo_cfg.crop_height <= 0 ||
            stereo_cfg.original_width <= 0 || stereo_cfg.original_height <= 0) {
            RCLCPP_ERROR(node->get_logger(),
                         "Config '%s' must provide positive original_width/original_height and crop_width/crop_height",
                         cfg.viewer_name.c_str());
            rclcpp::shutdown();
            return 1;
        }

        const std::string unixfd_upload_chain = get_unixfd_upload_chain();
        const std::string unixfd_socket_path = resolve_unixfd_socket_path(stereo_cfg);
        if (stereo_cfg.has_unixfd_socket_path) {
            if (stereo_cfg.unixfd_socket_path.empty()) {
                RCLCPP_INFO(node->get_logger(), "unixfd publish path (default): %s", unixfd_socket_path.c_str());
            } else {
                RCLCPP_INFO(node->get_logger(), "unixfd publish path: %s", unixfd_socket_path.c_str());
            }
            RCLCPP_INFO(node->get_logger(), "unixfd export chain: %s", unixfd_upload_chain.c_str());
        } else {
            RCLCPP_INFO(node->get_logger(), "unixfd publish disabled (stereo.unixfd_socket_path is set to empty)");
        }

        pipeline_string = build_pipeline_string(stereo_cfg, overlay_available);
        break;
    }

    const std::string camera_topic = "/" + console_name + "/camera";
    const std::string clutch_topic = "/" + console_name + "/clutch";
    RCLCPP_INFO(node->get_logger(), "Console topics: camera=%s clutch=%s", camera_topic.c_str(), clutch_topic.c_str());

    const auto latch_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto camera_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        camera_topic,
        latch_qos,
        [overlay_state](const sensor_msgs::msg::Joy::SharedPtr msg) {
            on_camera_joy(msg, overlay_state);
        }
    );

    auto clutch_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        clutch_topic,
        latch_qos,
        [overlay_state](const sensor_msgs::msg::Joy::SharedPtr msg) {
            on_clutch_joy(msg, overlay_state);
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
            g_signal_connect(overlay, "caps-changed", G_CALLBACK(on_overlay_caps_changed), overlay_state.get());
            g_signal_connect(overlay, "draw", G_CALLBACK(on_overlay_draw), overlay_state.get());
            gst_object_unref(overlay);
        } else {
            RCLCPP_WARN(node->get_logger(), "Unable to find 'stereo_overlay' element in pipeline; dVRK status overlay is disabled");
        }
    }

    g_main_loop = g_main_loop_new(nullptr, FALSE);
    g_unix_signal_add(SIGINT, on_sigint, nullptr);
    g_unix_signal_add(SIGTERM, on_sigint, nullptr);
    g_timeout_add(20, on_ros_spin, node.get());

    rclcpp::spin_some(node->get_node_base_interface());

    (void)camera_sub;
    (void)clutch_sub;

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

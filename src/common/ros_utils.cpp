#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <ctime>
#include <gst/video/video.h>
#include <gst/app/gstappsink.h>
#include <sensor_msgs/image_encodings.hpp>
#include "ros_utils.hpp"

namespace sv {

std::string ros_image_target_name(RosImageTarget target) {
    switch (target) {
    case RosImageTarget::Left: return "left";
    case RosImageTarget::Right: return "right";
    case RosImageTarget::Stereo: return "stereo";
    case RosImageTarget::Mono: return "mono";
    }
    return "unknown";
}

std::string ros_sink_name(RosImageTarget target) {
    switch (target) {
    case RosImageTarget::Left: return "__ros_left_sink__";
    case RosImageTarget::Right: return "__ros_right_sink__";
    case RosImageTarget::Stereo: return "__ros_stereo_sink__";
    case RosImageTarget::Mono: return "__ros_mono_sink__";
    }
    return "__ros_unknown_sink__";
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

bool has_ros_target(const std::vector<RosImageTarget>& targets, RosImageTarget target) {
    return std::find(targets.begin(), targets.end(), target) != targets.end();
}

bool parse_ros_image_target(const std::string& value, RosImageTarget& target) {
    std::string normalized = value;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (normalized == "left") { target = RosImageTarget::Left; return true; }
    if (normalized == "right") { target = RosImageTarget::Right; return true; }
    if (normalized == "stereo") { target = RosImageTarget::Stereo; return true; }
    if (normalized == "mono") { target = RosImageTarget::Mono; return true; }
    return false;
}

bool parse_ros_image_publishers(const std::vector<std::string>& publisher_names,
                                const rclcpp::Logger& logger,
                                std::vector<RosImageTarget>& targets) {
    targets.clear();
    for (const auto& name : publisher_names) {
        RosImageTarget target;
        if (parse_ros_image_target(name, target)) {
            targets.push_back(target);
        } else {
            RCLCPP_ERROR(logger, "Invalid ros_image_publishers entry '%s'. Allowed: left, right, stereo, mono", name.c_str());
            return false;
        }
    }
    return true;
}

void on_new_ros_image_sample(GstElement* sink, gpointer user_data) {
    auto* ctx = static_cast<RosImagePublisherContext*>(user_data);
    if (!ctx) return;

    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample) return;

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    GstVideoInfo video_info;

    if (buffer && caps && gst_video_info_from_caps(&video_info, caps)) {
        GstVideoFrame frame;
        if (gst_video_frame_map(&frame, &video_info, buffer, GST_MAP_READ)) {
            auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
            auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();

            timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            image_msg->header.stamp.sec = ts.tv_sec;
            image_msg->header.stamp.nanosec = ts.tv_nsec;
            image_msg->header.frame_id = ctx->frame_id;

            image_msg->height = GST_VIDEO_INFO_HEIGHT(&video_info);
            image_msg->width = GST_VIDEO_INFO_WIDTH(&video_info);
            image_msg->encoding = sensor_msgs::image_encodings::RGB8;
            image_msg->step = image_msg->width * 3;

            const size_t size = image_msg->step * image_msg->height;
            image_msg->data.resize(size);

            const uint8_t* src = static_cast<uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
            const int src_stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
            uint8_t* dst = image_msg->data.data();

            if (static_cast<int>(image_msg->step) == src_stride) {
                std::memcpy(dst, src, size);
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

            ctx->publisher.publish(image_msg, camera_info_msg);
            gst_video_frame_unmap(&frame);
        }
    }
    gst_sample_unref(sample);
}

gboolean on_bus_message(GstBus*, GstMessage* message, gpointer user_data) {
    if (!message) return G_SOURCE_CONTINUE;
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS) {
        if (user_data) g_main_loop_quit(static_cast<GMainLoop*>(user_data));
    } else if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
        GError* err = nullptr;
        gchar* dbg = nullptr;
        gst_message_parse_error(message, &err, &dbg);
        std::cerr << "GStreamer error: " << (err ? err->message : "unknown") << std::endl;
        if (dbg) { std::cerr << "Debug details: " << dbg << std::endl; g_free(dbg); }
        if (err) g_error_free(err);
        if (user_data) g_main_loop_quit(static_cast<GMainLoop*>(user_data));
    }
    return G_SOURCE_CONTINUE;
}

// Use static/persistent storage for subscriptions to keep them alive
static std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
static auto following_subscribers_cache = std::make_shared<std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>>();
static auto measured_cp_subscribers_cache = std::make_shared<std::unordered_map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>>();
static auto tool_type_subscribers_cache = std::make_shared<std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr>>();
static auto active_teleops = std::make_shared<std::unordered_set<std::string>>();
static auto latest_teleop_by_mtm = std::make_shared<std::unordered_map<std::string, std::string>>();

void setup_dvrk_overlay_subscriptions(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<OverlayState> overlay_state,
    const std::string& console_namespace
) {
    const std::string camera_topic = "/" + console_namespace + "/camera";
    const std::string clutch_topic = "/" + console_namespace + "/clutch";
    const std::string teleop_selected_topic = "/" + console_namespace + "/teleop/selected";
    const std::string teleop_unselected_topic = "/" + console_namespace + "/teleop/unselected";

    const auto latch_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto measured_cp_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto persistent_event_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto teleop_selected_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().transient_local();

    subs.push_back(node->create_subscription<sensor_msgs::msg::Joy>(
        camera_topic, latch_qos, [overlay_state](const sensor_msgs::msg::Joy::SharedPtr msg) { sv::on_camera_joy(msg, overlay_state); }));

    subs.push_back(node->create_subscription<sensor_msgs::msg::Joy>(
        clutch_topic, latch_qos, [overlay_state](const sensor_msgs::msg::Joy::SharedPtr msg) { sv::on_clutch_joy(msg, overlay_state); }));

    subs.push_back(node->create_subscription<std_msgs::msg::String>(
        teleop_selected_topic,
        teleop_selected_qos,
        [node, overlay_state, latch_qos, measured_cp_qos, persistent_event_qos](const std_msgs::msg::String::SharedPtr msg) {
            if (!msg) return;
            std::string mtm_name, arm_name;
            sv::TeleopSide side;
            int psm_number = 0;
            bool is_camera_teleop = false;
            if (!sv::parse_teleop_name(msg->data, mtm_name, side, psm_number, &arm_name, &is_camera_teleop)) return;

            const std::string teleop_name = msg->data;
            const auto latest_it = latest_teleop_by_mtm->find(mtm_name);
            if (latest_it != latest_teleop_by_mtm->end() && latest_it->second != teleop_name) {
                const std::string previous_teleop = latest_it->second;
                active_teleops->erase(previous_teleop);
                auto previous_msg = std::make_shared<std_msgs::msg::String>();
                previous_msg->data = previous_teleop;
                sv::on_teleop_unselected(previous_msg, overlay_state);
            }
            (*latest_teleop_by_mtm)[mtm_name] = teleop_name;
            active_teleops->insert(teleop_name);
            sv::on_teleop_selected(msg, overlay_state);

            if (following_subscribers_cache->find(teleop_name) == following_subscribers_cache->end()) {
                const std::string following_topic = "/" + teleop_name + "/following";
                auto following_sub = node->create_subscription<std_msgs::msg::Bool>(
                    following_topic, latch_qos, [overlay_state, teleop_name](const std_msgs::msg::Bool::SharedPtr following_msg) {
                        if (active_teleops->find(teleop_name) == active_teleops->end()) return;
                        sv::on_teleop_following(teleop_name, following_msg, overlay_state);
                    });
                (*following_subscribers_cache)[teleop_name] = following_sub;
            }

            if (!arm_name.empty() && measured_cp_subscribers_cache->find(arm_name) == measured_cp_subscribers_cache->end()) {
                const std::string measured_cp_topic = "/" + arm_name + "/measured_cp";
                auto measured_cp_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                    measured_cp_topic, measured_cp_qos, [overlay_state, arm_name](const geometry_msgs::msg::PoseStamped::SharedPtr measured_cp_msg) {
                        sv::on_teleop_measured_cp(arm_name, measured_cp_msg, overlay_state);
                    });
                (*measured_cp_subscribers_cache)[arm_name] = measured_cp_sub;
            }

            if (!is_camera_teleop && psm_number > 0) {
                const std::string psm_name = "PSM" + std::to_string(psm_number);
                if (tool_type_subscribers_cache->find(psm_name) == tool_type_subscribers_cache->end()) {
                    const std::string tool_type_topic = "/" + psm_name + "/tool_type";
                    auto tool_type_sub = node->create_subscription<std_msgs::msg::String>(
                        tool_type_topic, persistent_event_qos, [overlay_state, psm_name](const std_msgs::msg::String::SharedPtr tool_type_msg) {
                            sv::on_teleop_tool_type(psm_name, tool_type_msg, overlay_state);
                        });
                    (*tool_type_subscribers_cache)[psm_name] = tool_type_sub;
                }
            }
        }
    ));

    subs.push_back(node->create_subscription<std_msgs::msg::String>(
        teleop_unselected_topic,
        latch_qos,
        [node, overlay_state](const std_msgs::msg::String::SharedPtr msg) {
            if (!msg) return;
            std::string mtm_name;
            sv::TeleopSide side;
            int psm_number = 0;
            const bool parsed = sv::parse_teleop_name(msg->data, mtm_name, side, psm_number);
            sv::on_teleop_unselected(msg, overlay_state);
            active_teleops->erase(msg->data);
            if (parsed) {
                const auto latest_it = latest_teleop_by_mtm->find(mtm_name);
                if (latest_it != latest_teleop_by_mtm->end() && latest_it->second == msg->data) {
                    latest_teleop_by_mtm->erase(latest_it);
                }
            }
        }
    ));
}

void cleanup_dvrk_overlay_subscriptions() {
    subs.clear();
    following_subscribers_cache->clear();
    measured_cp_subscribers_cache->clear();
    tool_type_subscribers_cache->clear();
    active_teleops->clear();
    latest_teleop_by_mtm->clear();
}

} // namespace sv

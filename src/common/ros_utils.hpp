#ifndef DVRK_DISPLAY_COMMON_ROS_UTILS_HPP
#define DVRK_DISPLAY_COMMON_ROS_UTILS_HPP

#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include "../dvrk_display/overlay.hpp"

namespace sv {

enum class RosImageTarget {
    Left,
    Right,
    Stereo,
    Mono
};

struct RosImagePublisherContext {
    RosImageTarget target = RosImageTarget::Mono;
    std::string topic_base;
    std::string frame_id;
    image_transport::CameraPublisher publisher;
};

std::string ros_image_target_name(RosImageTarget target);
std::string ros_sink_name(RosImageTarget target);
std::string trim_topic_tokens(std::string value);

bool has_ros_target(const std::vector<RosImageTarget>& targets, RosImageTarget target);

bool parse_ros_image_publishers(const std::vector<std::string>& publisher_names,
                                const rclcpp::Logger& logger,
                                std::vector<RosImageTarget>& targets);

void on_new_ros_image_sample(GstElement* sink, gpointer user_data);
gboolean on_bus_message(GstBus* bus, GstMessage* message, gpointer user_data);

// Subscriber setup for OverlayState
void setup_dvrk_overlay_subscriptions(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<OverlayState> overlay_state,
    const std::string& console_namespace
);

void cleanup_dvrk_overlay_subscriptions();

} // namespace sv

#endif

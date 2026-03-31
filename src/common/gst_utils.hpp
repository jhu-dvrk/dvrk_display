#ifndef DVRK_DISPLAY_COMMON_GST_UTILS_HPP
#define DVRK_DISPLAY_COMMON_GST_UTILS_HPP

#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "config.hpp"

namespace sv {

bool validate_pipeline(const std::string& stream, const rclcpp::Logger& logger, const std::string& name);
void warn_if_interlaced_stream(const std::string& stream, const rclcpp::Logger& logger, const std::string& name);
std::string resolve_unixfd_socket_path(const sv::AppConfig& cfg);
bool check_element_available(const std::string& element_name);
std::string get_unixfd_upload_chain();

} // namespace sv

#endif

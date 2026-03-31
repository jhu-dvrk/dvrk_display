#include "gst_utils.hpp"
#include <gst/app/gstappsink.h>
#include <pwd.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <cstring>

namespace sv {

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

void warn_if_interlaced_stream(const std::string& stream, const rclcpp::Logger& logger, const std::string& name) {
    if (stream.empty()) {
        return;
    }

    const std::string probe_pipeline =
        stream +
        " ! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream"
        " ! appsink name=__caps_probe_sink__ sync=false async=false emit-signals=false drop=true max-buffers=1";

    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(probe_pipeline.c_str(), &error);
    if (error != nullptr || pipeline == nullptr) {
        if (error != nullptr) {
            RCLCPP_WARN(logger,
                        "Unable to probe caps for '%s' stream: %s",
                        name.c_str(),
                        error->message != nullptr ? error->message : "unknown error");
            g_error_free(error);
        }
        if (pipeline != nullptr) {
            gst_object_unref(pipeline);
        }
        return;
    }

    GstElement* probe_sink = gst_bin_get_by_name(GST_BIN(pipeline), "__caps_probe_sink__");
    if (probe_sink == nullptr) {
        gst_object_unref(pipeline);
        RCLCPP_WARN(logger, "Unable to probe caps for '%s' stream: missing probe sink", name.c_str());
        return;
    }

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(probe_sink), 2 * GST_SECOND);

    if (sample != nullptr) {
        GstCaps* caps = gst_sample_get_caps(sample);
        if (caps != nullptr && gst_caps_get_size(caps) > 0) {
            const GstStructure* structure = gst_caps_get_structure(caps, 0);
            const gchar* interlace_mode = gst_structure_get_string(structure, "interlace-mode");
            if (interlace_mode != nullptr && std::strcmp(interlace_mode, "progressive") != 0) {
                RCLCPP_WARN(logger,
                            "%s stream caps report interlace-mode='%s'. Consider adding deinterlace to this stream in the config.",
                            name.c_str(),
                            interlace_mode);
            }
        }
        gst_sample_unref(sample);
    }

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(probe_sink);
    gst_object_unref(pipeline);
}

std::string resolve_unixfd_socket_path(const sv::AppConfig& cfg) {
    if (!cfg.has_unixfd_socket_path) {
        return "";
    }

    if (!cfg.unixfd_socket_path.empty()) {
        return cfg.unixfd_socket_path;
    }

    const char* username = getenv("USER");
    if (!username) {
        struct passwd* pw = getpwuid(getuid());
        username = pw ? pw->pw_name : "unknown";
    }
    return "/tmp/dvrk_display_" + std::string(username) + ".sock";
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

} // namespace sv

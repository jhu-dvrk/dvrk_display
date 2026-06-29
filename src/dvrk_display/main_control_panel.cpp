#include <rclcpp/rclcpp.hpp>
#include <crtk_msgs/msg/operating_state.hpp>
#include <crtk_msgs/msg/string_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <glib-unix.h>
#include <glibmm/fileutils.h>
#include <glibmm/keyfile.h>
#include <glibmm/miscutils.h>
#include <gst/gst.h>
#include <gtkmm.h>
#include <gdk/gdkkeysyms.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <pwd.h>
#include <sstream>
#include <unistd.h>

#include "config.hpp"

// Configuration and State structures
struct ArmStatus {
    std::string name;
    std::string state;
    bool is_homed = false;
    bool is_busy = false;
    std::string tool_type;
    bool active = false;
};

struct ResetState {
    std::string arm_name;
    enum Stage { IDLE, WAITING_FOR_DISABLE, WAITING_FOR_ENABLE, WAITING_FOR_HOME, FINISHED } stage = IDLE;
    std::chrono::steady_clock::time_point last_command_time;
};

struct VideoSource {
    std::string label;
    std::string socket_path;
};

struct ControlPanelConfig {
    std::string name = "dvrk_display";
    std::string console = "console";
    std::vector<std::string> video_sources;
};

struct CommandLineOptions {
    std::string config_file;
    std::string console_name;
    bool console_override = false;
    std::vector<std::string> video_sources;
};

const char *get_username() {
    const char *username = getenv("USER");
    if (!username) {
        struct passwd *pw = getpwuid(getuid());
        username = pw ? pw->pw_name : "unknown";
    }
    return username;
}

std::string trim_video_source_prefix(std::string source) {
    if (!source.empty() && source.front() == ':') {
        source.erase(source.begin());
    }
    return source;
}

std::string video_source_label(const std::string& source) {
    const std::string normalized = trim_video_source_prefix(source);
    size_t last_slash = normalized.find_last_of('/');
    return (last_slash == std::string::npos) ? normalized : normalized.substr(last_slash + 1);
}

std::string resolve_video_source_path(const std::string& viewer_name,
                                      const std::string& source) {
    const std::string normalized = trim_video_source_prefix(source);
    if (normalized.find('/') != std::string::npos ||
        normalized.find(".sock") != std::string::npos) {
        return normalized;
    }
    return "/tmp/" + viewer_name + "_" + normalized + "_" +
           std::string(get_username()) + ".sock";
}

VideoSource make_video_source(const std::string& viewer_name,
                              const std::string& source) {
    return VideoSource{video_source_label(source),
                       resolve_video_source_path(viewer_name, source)};
}

void print_usage(const char *executable) {
    std::cerr << "Usage: " << executable
              << " [-c <config.json>] [-C <console>] [-s <video-source>]..."
              << std::endl;
    std::cerr << "  -c, --config   Control panel JSON config file" << std::endl;
    std::cerr << "  -C, --console  dVRK console namespace override" << std::endl;
    std::cerr << "  -s, --source   Video source socket path or stream name" << std::endl;
}

bool parse_arguments(int argc, char *argv[], CommandLineOptions &options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--ros-args") {
            break;
        }

        if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            if (!options.config_file.empty()) {
                std::cerr << "Error: multiple control panel config files are not supported."
                          << std::endl;
                return false;
            }
            options.config_file = argv[++i];
            continue;
        }

        if ((arg == "-C" || arg == "--console") && i + 1 < argc) {
            options.console_name = argv[++i];
            options.console_override = true;
            continue;
        }

        if ((arg == "-s" || arg == "--source") && i + 1 < argc) {
            options.video_sources.push_back(argv[++i]);
            continue;
        }

        if (arg == "-c" || arg == "--config" ||
            arg == "-C" || arg == "--console" ||
            arg == "-s" || arg == "--source") {
            std::cerr << "Error: missing value for argument '" << arg << "'."
                      << std::endl;
            return false;
        }

        std::cerr << "Error: unknown argument '" << arg << "'." << std::endl;
        return false;
    }
    return true;
}

bool load_control_panel_config(const std::string& path, ControlPanelConfig& config) {
    if (!std::filesystem::exists(path)) {
        std::cerr << "Config file does not exist: " << path << std::endl;
        return false;
    }

    Json::Value root;
    if (!sv::Config::load_from_file(path, root)) {
        return false;
    }
    if (!root.isObject()) {
        std::cerr << "Configuration error: control panel config root must be a JSON object."
                  << std::endl;
        return false;
    }

    if (root.isMember("name")) {
        if (!root["name"].isString()) {
            std::cerr << "Configuration error: 'name' must be a string."
                      << std::endl;
            return false;
        }
        if (!root["name"].asString().empty()) {
            config.name = root["name"].asString();
        }
    }

    if (root.isMember("console")) {
        if (!root["console"].isString()) {
            std::cerr << "Configuration error: 'console' must be a string."
                      << std::endl;
            return false;
        }
        if (!root["console"].asString().empty()) {
            config.console = root["console"].asString();
        }
    }

    if (!root.isMember("video_source") || !root["video_source"].isArray()) {
        std::cerr << "Configuration error: 'video_source' must be an array of strings."
                  << std::endl;
        return false;
    }

    config.video_sources.clear();
    for (const auto& item : root["video_source"]) {
        if (!item.isString()) {
            std::cerr << "Configuration error: all 'video_source' entries must be strings."
                      << std::endl;
            return false;
        }
        const std::string source = item.asString();
        if (!source.empty()) {
            config.video_sources.push_back(source);
        }
    }

    return true;
}

// UI and ROS2 Main Window
class ControlPanelWindow : public Gtk::Window {
public:
    ControlPanelWindow(std::shared_ptr<rclcpp::Node> node, 
                      const std::string& settings_name,
                      const std::string& console_name,
                      const std::vector<VideoSource>& video_sources)
        : m_node(node), m_settings_name(settings_name), m_console_name(console_name),
          m_video_sources(video_sources),
          m_main_paned(Gtk::ORIENTATION_HORIZONTAL),
          m_left_pane(Gtk::ORIENTATION_VERTICAL, 10),
          m_right_pane(Gtk::ORIENTATION_VERTICAL, 5),
          m_arms_box(Gtk::ORIENTATION_VERTICAL, 6),
          m_teleops_box(Gtk::ORIENTATION_VERTICAL, 6),
          m_reset_timer_active(false),
          m_teleop_master_enabled(false),
          m_dvrk_powered_on(false),
          m_dark_mode(false)
    {
        set_title("dVRK Control Panel");
        set_default_size(1024, 600);

        // Apply sleek styling and dynamic CSS provider
        setup_styles();

        // Build left control panel structure
        setup_left_pane();

        // Setup Main Resizable Layout
        if (!m_video_sources.empty()) {
            m_right_pane.set_hexpand(true);
            m_right_pane.set_vexpand(true);
            m_main_paned.pack1(m_left_pane, false, false); // Keep left controls fixed size on window resize
            m_main_paned.pack2(m_right_pane, true, false);  // Let right video expand to fill extra space
            m_main_paned.set_position(380); // Heuristic initial split: 380px for controls, rest for video
            add(m_main_paned);
        } else {
            add(m_left_pane);
        }

        // Initialize ROS2 Publishers and Subscribers
        setup_ros();

        // Periodically tick UI elements (e.g. reset state machines, ROS updates)
        m_ui_tick_connection = Glib::signal_timeout().connect(sigc::mem_fun(*this, &ControlPanelWindow::on_ui_tick), 100);

        // Show window children first to realize widgets
        show_all_children();

        // Build right video panel structure and start pipelines AFTER the layout is attached and shown
        setup_right_pane();

        load_persisted_display_settings();
        if (m_have_persisted_display_settings) {
            Glib::signal_idle().connect_once([this]() {
                apply_window_display_settings();
            });
        }
        m_monitor_poll_connection = Glib::signal_timeout().connect(
            sigc::mem_fun(*this, &ControlPanelWindow::poll_window_monitor), 1000);
    }

    virtual ~ControlPanelWindow() {
        if (m_ui_tick_connection.connected()) {
            m_ui_tick_connection.disconnect();
        }
        if (m_monitor_poll_connection.connected()) {
            m_monitor_poll_connection.disconnect();
        }

        close_video_source();
    }

protected:
    // Handle window configuration events (resize, maximize, drag to different monitors)
    bool on_configure_event(GdkEventConfigure* configure_event) override {
        update_dynamic_css(configure_event->width, configure_event->height);
        return Gtk::Window::on_configure_event(configure_event);
    }

    bool on_delete_event(GdkEventAny* /* any_event */) override {
        save_persisted_display_settings();
        hide();
        return true;
    }

    bool on_window_state_event(GdkEventWindowState* event) override {
        const bool handled = Gtk::Window::on_window_state_event(event);
        if ((event->changed_mask & GDK_WINDOW_STATE_FULLSCREEN) != 0) {
            m_window_fullscreen =
                (event->new_window_state & GDK_WINDOW_STATE_FULLSCREEN) != 0;
            save_persisted_display_settings();
        }
        return handled;
    }

private:
    void setup_styles() {
        m_css_provider = Gtk::CssProvider::create();
        auto screen = Gdk::Screen::get_default();
        Gtk::StyleContext::add_provider_for_screen(screen, m_css_provider, GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);

        // Parse colors based on theme
        if (m_dark_mode) {
            m_color_green.set("#2ec4b6");
            m_color_red.set("#e71d36");
            m_color_orange.set("#ff9f1c");
        } else {
            m_color_green.set("#00875a");
            m_color_red.set("#de350b");
            m_color_orange.set("#ff8c00");
        }

        // Initial default style load
        update_dynamic_css(1024, 600);
    }

    void update_dynamic_css(int width, int height) {
        (void)width;
        // Font sizes and dimension limits based on actual window height
        int frame_title_size = std::max(12, std::min(18, height / 45));
        int arm_name_size = std::max(14, std::min(22, height / 36));
        int status_size = std::max(12, std::min(18, height / 40));
        int tool_size = std::max(11, std::min(16, height / 48));
        int reset_btn_size = std::max(30, std::min(48, height / 16));
        int reset_font_size = std::max(14, std::min(22, height / 24));
        int padding_size = std::max(4, std::min(12, height / 90));
        int margin_size = std::max(3, std::min(8, height / 120));

        int slider_handle_size = std::max(24, std::min(36, height / 22));

        std::stringstream ss;
        if (m_dark_mode) {
            ss << "window { background-color: #121214; color: #e2e2e9; font-family: 'Inter', 'Sans', sans-serif; }\n"
               << "frame { border: 1px solid #2d2d34; border-radius: 10px; margin: " << margin_size << "px; padding: " << padding_size << "px; background-color: #1a1a1e; }\n"
               << "frame label { font-weight: bold; color: #00adb5; font-size: " << frame_title_size << "px; }\n"
               << "button.reset-btn { background-color: #2a2b36; color: #ff5722; border: 1px solid #3f4052; border-radius: 50%; padding: 0px; font-size: " << reset_font_size << "px; min-width: " << reset_btn_size << "px; min-height: " << reset_btn_size << "px; }\n"
               << "button.reset-btn:hover { background-color: #3f4052; color: #ff784e; }\n"
               << "label.arm-name { font-weight: bold; font-size: " << arm_name_size << "px; color: #ffffff; }\n"
               << "label.arm-status { font-size: " << status_size << "px; font-weight: bold; }\n"
               << "label.arm-tool { font-size: " << tool_size << "px; color: #00adb5; font-style: italic; }\n"
               << "scale slider { min-height: " << slider_handle_size << "px; min-width: " << slider_handle_size << "px; background-color: #00adb5; border-radius: 50%; }\n"
               << "scale trough { border-radius: 8px; background-color: #2d2d34; }\n"
               << "combobox { background-color: #1a1a1e; color: #ffffff; }\n"
               << "combobox * { background-color: #1a1a1e; color: #ffffff; }\n"
               << "combobox { border: 1px solid #2d2d34; padding: 6px; font-size: " << status_size << "px; }\n"
               << "label.slider-name { font-weight: bold; font-size: " << status_size << "px; color: #e2e2e9; }\n"
               << "label.slider-val { font-size: " << status_size << "px; color: #e2e2e9; }\n"
               << "menu { background-color: #1a1a1e; border: 1px solid #2d2d34; border-radius: 8px; padding: 6px; }\n"
               << "menuitem { padding: 8px 16px; border-radius: 6px; margin: 2px 4px; }\n"
               << "menuitem label { font-family: 'Inter', 'Sans', sans-serif; font-weight: 500; font-size: 16px; }\n"
               << "menuitem:hover { color: #ffffff; background-color: #00adb5; }\n"
               << "menuitem:hover label { color: #ffffff; }\n";
        } else {
            ss << "window { background-color: #f4f5f8; color: #2d3748; font-family: 'Inter', 'Sans', sans-serif; }\n"
               << "frame { border: 1px solid #cbd5e0; border-radius: 10px; margin: " << margin_size << "px; padding: " << padding_size << "px; background-color: #ffffff; }\n"
               << "frame label { font-weight: bold; color: #008080; font-size: " << frame_title_size << "px; }\n"
               << "button.reset-btn { background-color: #edf2f7; color: #e53e3e; border: 1px solid #cbd5e0; border-radius: 50%; padding: 0px; font-size: " << reset_font_size << "px; min-width: " << reset_btn_size << "px; min-height: " << reset_btn_size << "px; }\n"
               << "button.reset-btn:hover { background-color: #e2e8f0; color: #c53030; }\n"
               << "label.arm-name { font-weight: bold; font-size: " << arm_name_size << "px; color: #1a202c; }\n"
               << "label.arm-status { font-size: " << status_size << "px; font-weight: bold; }\n"
               << "label.arm-tool { font-size: " << tool_size << "px; color: #008080; font-style: italic; }\n"
               << "scale slider { min-height: " << slider_handle_size << "px; min-width: " << slider_handle_size << "px; background-color: #008080; border-radius: 50%; }\n"
               << "scale trough { border-radius: 8px; background-color: #edf2f7; }\n"
               << "combobox { background-color: #ffffff; color: #2d3748; }\n"
               << "combobox * { background-color: #ffffff; color: #2d3748; }\n"
               << "combobox { border: 1px solid #cbd5e0; padding: 6px; font-size: " << status_size << "px; }\n"
               << "label.slider-name { font-weight: bold; font-size: " << status_size << "px; color: #2d3748; }\n"
               << "label.slider-val { font-size: " << status_size << "px; color: #2d3748; }\n"
               << "menu { background-color: #ffffff; border: 1px solid #cbd5e0; border-radius: 8px; padding: 6px; }\n"
               << "menuitem { padding: 8px 16px; border-radius: 6px; margin: 2px 4px; }\n"
               << "menuitem label { font-family: 'Inter', 'Sans', sans-serif; font-weight: 500; font-size: 16px; }\n"
               << "menuitem:hover { color: #ffffff; background-color: #008080; }\n"
               << "menuitem:hover label { color: #ffffff; }\n";
        }

        try {
            m_css_provider->load_from_data(ss.str());
        } catch (const Gtk::CssProviderError& ex) {
            std::cerr << "Gtk::CssProviderError: " << ex.what() << std::endl;
            std::cerr << "CSS Content:\n" << ss.str() << std::endl;
            throw;
        } catch (const std::exception& ex) {
            std::cerr << "CSS Standard Error: " << ex.what() << std::endl;
            throw;
        }
    }

    void setup_left_pane() {
        // 1. Arms Status Frame (No label name)
        m_arms_box.set_margin_left(16);
        m_arms_box.set_margin_right(16);
        m_arms_box.set_margin_top(12);
        m_arms_box.set_margin_bottom(12);
        m_arms_frame.add(m_arms_box);
        m_left_pane.pack_start(m_arms_frame, Gtk::PACK_EXPAND_WIDGET, 0);

        // Master dVRK power button at the top of Arms Frame
        Gtk::Box* dvrk_master_layout = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 10));
        dvrk_master_layout->set_margin_bottom(12);

        m_dvrk_power_btn = Gtk::manage(new Gtk::Button("⏻"));
        m_dvrk_power_btn->get_style_context()->add_class("reset-btn");
        m_dvrk_power_btn->set_valign(Gtk::ALIGN_CENTER);
        m_dvrk_power_btn->set_margin_right(12);

        Gtk::Box* dvrk_master_text_box = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 2));
        Gtk::Box* dvrk_master_first_line = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 6));

        Gtk::Label* dvrk_title = Gtk::manage(new Gtk::Label("dVRK", Gtk::ALIGN_START));
        dvrk_title->get_style_context()->add_class("arm-name");

        m_dvrk_power_status = Gtk::manage(new Gtk::Label("DISABLED", Gtk::ALIGN_START));
        m_dvrk_power_status->get_style_context()->add_class("arm-status");
        m_dvrk_power_status->override_color(m_color_red);

        dvrk_master_first_line->pack_start(*dvrk_title, Gtk::PACK_SHRINK);
        dvrk_master_first_line->pack_start(*m_dvrk_power_status, Gtk::PACK_SHRINK);

        dvrk_master_text_box->pack_start(*dvrk_master_first_line, Gtk::PACK_SHRINK);

        dvrk_master_layout->pack_start(*m_dvrk_power_btn, Gtk::PACK_SHRINK, 0);
        dvrk_master_layout->pack_start(*dvrk_master_text_box, Gtk::PACK_EXPAND_WIDGET, 0);

        m_arms_box.pack_start(*dvrk_master_layout, Gtk::PACK_SHRINK, 0);

        m_dvrk_power_btn->signal_clicked().connect(sigc::mem_fun(*this, &ControlPanelWindow::on_dvrk_power_clicked));

        // Instantiate rows for standard dVRK arms (PSM1-3, ECM, MTML-R)
        std::vector<std::string> arm_names = {"PSM1", "PSM2", "PSM3", "ECM", "MTML", "MTMR"};
        for (const auto& name : arm_names) {
            create_arm_row(name);
        }

        // 2. Teleops List Frame (No label name)
        m_teleops_box.set_margin_left(16);
        m_teleops_box.set_margin_right(16);
        m_teleops_box.set_margin_top(12);
        m_teleops_box.set_margin_bottom(12);
        m_teleops_frame.add(m_teleops_box);
        m_left_pane.pack_start(m_teleops_frame, Gtk::PACK_EXPAND_WIDGET, 0);

        // Master enable/disable button inside Teleoperation Frame
        Gtk::Box* master_layout = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 10));
        master_layout->set_margin_bottom(12);

        m_teleop_enable_btn = Gtk::manage(new Gtk::Button("⏻"));
        m_teleop_enable_btn->get_style_context()->add_class("reset-btn");
        m_teleop_enable_btn->set_valign(Gtk::ALIGN_CENTER);
        m_teleop_enable_btn->set_margin_right(12);

        Gtk::Box* master_text_box = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 2));

        // Master first line: Title + ENABLED/DISABLED State (Removed "Master" word after "Teleoperation")
        Gtk::Box* master_first_line = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 6));
        Gtk::Label* master_title = Gtk::manage(new Gtk::Label("Teleoperation", Gtk::ALIGN_START));
        master_title->get_style_context()->add_class("arm-name");
        m_teleop_enable_status = Gtk::manage(new Gtk::Label("DISABLED", Gtk::ALIGN_START));
        m_teleop_enable_status->get_style_context()->add_class("arm-status");
        m_teleop_enable_status->override_color(m_color_red);

        master_first_line->pack_start(*master_title, Gtk::PACK_SHRINK);
        master_first_line->pack_start(*m_teleop_enable_status, Gtk::PACK_SHRINK);

        master_text_box->pack_start(*master_first_line, Gtk::PACK_SHRINK);

        master_layout->pack_start(*m_teleop_enable_btn, Gtk::PACK_SHRINK, 0);
        master_layout->pack_start(*master_text_box, Gtk::PACK_EXPAND_WIDGET, 0);

        m_teleops_box.pack_start(*master_layout, Gtk::PACK_SHRINK, 0);

        m_teleop_enable_btn->signal_clicked().connect(sigc::mem_fun(*this, &ControlPanelWindow::on_master_enable_clicked));

        // 3. System Frame (Scale & Volume) (No label name)
        Gtk::Grid* system_grid = Gtk::manage(new Gtk::Grid());
        system_grid->set_row_spacing(10);
        system_grid->set_column_spacing(10);
        system_grid->set_margin_left(16);
        system_grid->set_margin_right(16);
        system_grid->set_margin_top(12);
        system_grid->set_margin_bottom(12);

        // Scale components
        Gtk::Label* scale_label = Gtk::manage(new Gtk::Label("Scale", Gtk::ALIGN_START));
        scale_label->get_style_context()->add_class("slider-name");
        m_scale_slider = Gtk::manage(new Gtk::Scale(Gtk::ORIENTATION_HORIZONTAL));
        m_scale_slider->set_range(0.0, 1.0);
        m_scale_slider->set_increments(0.1, 0.1);
        m_scale_slider->set_digits(1);
        m_scale_slider->set_draw_value(false); // Remove value above slider
        m_scale_slider->set_value(0.5);
        m_scale_slider->set_hexpand(true);
        m_scale_slider->set_valign(Gtk::ALIGN_CENTER);

        m_scale_value_label = Gtk::manage(new Gtk::Label("0.5"));
        m_scale_value_label->get_style_context()->add_class("slider-val");
        m_scale_value_label->set_width_chars(4);
        m_scale_value_label->set_valign(Gtk::ALIGN_CENTER);

        system_grid->attach(*scale_label, 0, 0, 1, 1);
        system_grid->attach(*m_scale_slider, 1, 0, 1, 1);
        system_grid->attach(*m_scale_value_label, 2, 0, 1, 1);

        // Volume components
        Gtk::Label* volume_label = Gtk::manage(new Gtk::Label("Volume", Gtk::ALIGN_START));
        volume_label->get_style_context()->add_class("slider-name");
        m_volume_slider = Gtk::manage(new Gtk::Scale(Gtk::ORIENTATION_HORIZONTAL));
        m_volume_slider->set_range(0.0, 1.0);
        m_volume_slider->set_increments(0.05, 0.1);
        m_volume_slider->set_digits(2);
        m_volume_slider->set_draw_value(false); // Remove value above slider
        m_volume_slider->set_value(0.5);
        m_volume_slider->set_hexpand(true);
        m_volume_slider->set_valign(Gtk::ALIGN_CENTER);

        m_volume_value_label = Gtk::manage(new Gtk::Label("50%"));
        m_volume_value_label->get_style_context()->add_class("slider-val");
        m_volume_value_label->set_width_chars(5);
        m_volume_value_label->set_valign(Gtk::ALIGN_CENTER);

        system_grid->attach(*volume_label, 0, 1, 1, 1);
        system_grid->attach(*m_volume_slider, 1, 1, 1, 1);
        system_grid->attach(*m_volume_value_label, 2, 1, 1, 1);

        // Add Wrench Settings button with margin space
        m_wrench_btn = Gtk::manage(new Gtk::Button("🔧"));
        m_wrench_btn->get_style_context()->add_class("reset-btn");
        m_wrench_btn->set_valign(Gtk::ALIGN_CENTER);
        m_wrench_btn->set_halign(Gtk::ALIGN_CENTER);
        m_wrench_btn->set_margin_left(16);
        m_wrench_btn->signal_clicked().connect(sigc::mem_fun(*this, &ControlPanelWindow::on_wrench_clicked));

        system_grid->attach(*m_wrench_btn, 3, 0, 1, 2);

        m_system_frame.add(*system_grid);
        m_left_pane.pack_start(m_system_frame, Gtk::PACK_SHRINK, 0);

        // Connect slider signals
        m_scale_connection = m_scale_slider->signal_value_changed().connect(
            sigc::mem_fun(*this, &ControlPanelWindow::on_scale_slider_changed));
        m_volume_connection = m_volume_slider->signal_value_changed().connect(
            sigc::mem_fun(*this, &ControlPanelWindow::on_volume_slider_changed));

        // Initialize user interaction lock times
        m_scale_last_change_time = std::chrono::steady_clock::now() - std::chrono::seconds(5);
        m_volume_last_change_time = std::chrono::steady_clock::now() - std::chrono::seconds(5);
        m_last_scale_request = m_scale_slider->get_value();
        m_last_volume_request = m_volume_slider->get_value();
    }

    void create_arm_row(const std::string& name) {
        ArmRow row;
        row.container = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 10));
        row.container->set_margin_bottom(4);
        row.container->set_margin_left(28); // Indent to show hierarchy under master button

        row.reset_button = Gtk::manage(new Gtk::Button("⟲"));
        row.reset_button->get_style_context()->add_class("reset-btn");
        row.reset_button->set_valign(Gtk::ALIGN_CENTER);
        row.reset_button->set_margin_right(12);
        row.reset_button->signal_clicked().connect([this, name]() { trigger_arm_reset(name); });

        Gtk::Box* text_box = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 2));

        // First Line: Name + Main state
        Gtk::Box* first_line = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 6));
        row.name_label = Gtk::manage(new Gtk::Label(name, Gtk::ALIGN_START));
        row.name_label->get_style_context()->add_class("arm-name");

        row.status_label = Gtk::manage(new Gtk::Label("OFFLINE", Gtk::ALIGN_START));
        row.status_label->get_style_context()->add_class("arm-status");

        first_line->pack_start(*row.name_label, Gtk::PACK_SHRINK);
        first_line->pack_start(*row.status_label, Gtk::PACK_SHRINK);

        // Second Line: Home/busy substate + Instrument Name (if PSM)
        Gtk::Box* second_line = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 6));
        row.substate_label = Gtk::manage(new Gtk::Label("", Gtk::ALIGN_START));
        row.substate_label->get_style_context()->add_class("arm-tool");
        second_line->pack_start(*row.substate_label, Gtk::PACK_SHRINK);

        if (name.rfind("PSM", 0) == 0) {
            row.tool_label = Gtk::manage(new Gtk::Label(" - No Instrument", Gtk::ALIGN_START));
            row.tool_label->get_style_context()->add_class("arm-tool");
            second_line->pack_start(*row.tool_label, Gtk::PACK_SHRINK);
        } else {
            row.tool_label = nullptr;
        }

        text_box->pack_start(*first_line, Gtk::PACK_SHRINK);
        text_box->pack_start(*second_line, Gtk::PACK_SHRINK);

        // Reset button placed on the left side
        row.container->pack_start(*row.reset_button, Gtk::PACK_SHRINK, 0);
        row.container->pack_start(*text_box, Gtk::PACK_EXPAND_WIDGET, 0);

        // Hide initially until active messages arrive
        row.container->set_visible(false);

        m_arms_box.pack_start(*row.container, Gtk::PACK_SHRINK, 0);
        m_arm_rows[name] = row;

        // Initialize status data
        m_arms_status[name] = ArmStatus{name, "OFFLINE", false, false, "", false};
    }

    void setup_right_pane() {
        if (m_video_sources.empty()) {
            return;
        }

        m_video_area.set_hexpand(true);
        m_video_area.set_vexpand(true);
        m_right_pane.pack_start(m_video_area, Gtk::PACK_EXPAND_WIDGET, 0);
        open_video_source(m_video_sources[0].socket_path);
        m_right_pane.show_all();
    }

    void close_video_source() {
        if (m_current_video_widget) {
            m_video_area.remove(*m_current_video_widget);
            m_current_video_widget = nullptr;
        }

        if (m_current_video_pipeline) {
            gst_element_set_state(m_current_video_pipeline, GST_STATE_NULL);
            gst_object_unref(m_current_video_pipeline);
            m_current_video_pipeline = nullptr;
        }

        m_current_video_path.clear();
    }

    bool open_video_source(const std::string& path) {
        if (path == m_current_video_path && m_current_video_pipeline) {
            return true;
        }

        close_video_source();

        std::string pipe_str = "unixfdsrc socket-path=" + path + " do-timestamp=true "
            "! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream "
            "! videoconvert ! gtksink name=sink";

        GError* error = nullptr;
        GstElement* pipeline = gst_parse_launch(pipe_str.c_str(), &error);
        if (error != nullptr || pipeline == nullptr) {
            std::cerr << "Failed to parse GStreamer pipeline for: " << path << std::endl;
            if (error) g_error_free(error);
            if (pipeline) gst_object_unref(pipeline);
            return false;
        }

        GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
        if (!sink) {
            std::cerr << "Failed to find gtksink for: " << path << std::endl;
            gst_object_unref(pipeline);
            return false;
        }

        GtkWidget* gtk_widget = nullptr;
        g_object_get(sink, "widget", &gtk_widget, nullptr);
        gst_object_unref(sink);

        if (!gtk_widget) {
            std::cerr << "Failed to get GTK widget for: " << path << std::endl;
            gst_object_unref(pipeline);
            return false;
        }

        Gtk::Widget* mm_widget = Glib::wrap(gtk_widget);
        mm_widget->set_hexpand(true);
        mm_widget->set_vexpand(true);
        m_video_area.pack_start(*mm_widget, Gtk::PACK_EXPAND_WIDGET, 0);
        mm_widget->show();
        g_object_unref(gtk_widget); // decrement reference returned by g_object_get

        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        m_current_video_widget = mm_widget;
        m_current_video_pipeline = pipeline;
        m_current_video_path = path;
        m_right_pane.show_all();
        return true;
    }

    void setup_ros() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();

        // Active teleop topic listeners
        std::string selected_topic = "/" + m_console_name + "/teleop/selected";
        std::string unselected_topic = "/" + m_console_name + "/teleop/unselected";

        m_teleop_selected_sub = m_node->create_subscription<std_msgs::msg::String>(
            selected_topic, qos, [this](const std_msgs::msg::String::SharedPtr msg) {
                handle_teleop_event(msg->data, true);
            });

        m_teleop_unselected_sub = m_node->create_subscription<std_msgs::msg::String>(
            unselected_topic, qos, [this](const std_msgs::msg::String::SharedPtr msg) {
                handle_teleop_event(msg->data, false);
            });

        // Master enabled status topic listeners
        std::string enabled_topic = "/" + m_console_name + "/teleop/enabled";
        m_teleop_enabled_sub = m_node->create_subscription<std_msgs::msg::Bool>(
            enabled_topic, qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                handle_teleop_enabled_event(msg->data);
            });

        std::string enable_topic = "/" + m_console_name + "/teleop/enable";
        m_teleop_enable_pub = m_node->create_publisher<std_msgs::msg::Bool>(enable_topic, 10);

        // dVRK System Power Publishers
        m_dvrk_home_pub = m_node->create_publisher<std_msgs::msg::Empty>("/system/home", 10);
        m_dvrk_power_off_pub = m_node->create_publisher<std_msgs::msg::Empty>("/system/power_off", 10);

        // Scale status subscriber and publisher
        std::string scale_topic = "/" + m_console_name + "/teleop/scale";
        m_scale_sub = m_node->create_subscription<std_msgs::msg::Float64>(
            scale_topic, qos, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                update_scale_ui(msg->data);
            });

        std::string set_scale_topic = "/" + m_console_name + "/teleop/set_scale";
        m_scale_pub = m_node->create_publisher<std_msgs::msg::Float64>(set_scale_topic, 10);

        // Volume status subscriber and publisher (full system topics)
        m_volume_sub = m_node->create_subscription<std_msgs::msg::Float64>(
            "/system/volume", qos, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                update_volume_ui(msg->data);
            });
        m_volume_pub = m_node->create_publisher<std_msgs::msg::Float64>("/system/set_volume", 10);

        // Teleoperation select/unselect publishers
        std::string select_topic = "/" + m_console_name + "/teleop/select";
        std::string unselect_topic = "/" + m_console_name + "/teleop/unselect";
        m_teleop_select_pub = m_node->create_publisher<std_msgs::msg::String>(select_topic, 10);
        m_teleop_unselect_pub = m_node->create_publisher<std_msgs::msg::String>(unselect_topic, 10);

        // Arm operating state, state commands, and tool type
        for (auto& pair : m_arms_status) {
            const std::string arm_name = pair.first;

            std::string op_topic = "/" + arm_name + "/operating_state";
            m_arm_subs[arm_name] = m_node->create_subscription<crtk_msgs::msg::OperatingState>(
                op_topic, qos, [this, arm_name](const crtk_msgs::msg::OperatingState::SharedPtr msg) {
                    handle_arm_operating_state(arm_name, msg);
                });

            std::string cmd_topic = "/" + arm_name + "/state_command";
            m_arm_pubs[arm_name] = m_node->create_publisher<crtk_msgs::msg::StringStamped>(cmd_topic, 10);

            if (arm_name.rfind("PSM", 0) == 0) {
                std::string tool_topic = "/" + arm_name + "/tool_type";
                m_tool_subs[arm_name] = m_node->create_subscription<std_msgs::msg::String>(
                    tool_topic, qos, [this, arm_name](const std_msgs::msg::String::SharedPtr msg) {
                        handle_arm_tool_type(arm_name, msg->data);
                    });
            }
        }
    }

    void update_dvrk_power_state(bool force_color_refresh = false) {
        bool all_enabled = true;
        bool any_active = false;
        for (const auto& pair : m_arms_status) {
            if (pair.second.active) {
                any_active = true;
                if (pair.second.state != "ENABLED") {
                    all_enabled = false;
                    break;
                }
            }
        }
        bool powered_on = any_active && all_enabled;
        if (m_dvrk_powered_on != powered_on || force_color_refresh) {
            m_dvrk_powered_on = powered_on;
            m_dvrk_power_status->set_text(m_dvrk_powered_on ? "ENABLED" : "DISABLED");
            if (m_dvrk_powered_on) {
                m_dvrk_power_status->override_color(m_color_green);
                m_dvrk_power_btn->override_color(m_color_green);
            } else {
                m_dvrk_power_status->override_color(m_color_red);
                m_dvrk_power_btn->override_color(m_color_red);
            }
        }
    }

    void handle_arm_operating_state(const std::string& arm_name, 
                                    const crtk_msgs::msg::OperatingState::SharedPtr msg) {
        auto& status = m_arms_status[arm_name];
        status.state = msg->state;
        status.is_homed = msg->is_homed;
        status.is_busy = msg->is_busy;
        status.active = true;

        update_dvrk_power_state();

        // UI updates
        auto& row = m_arm_rows[arm_name];
        row.status_label->set_text(status.state);

        // Color coding: Red when disabled, Green when enabled, Orange otherwise
        if (status.state == "ENABLED") {
            row.status_label->override_color(m_color_green);
        } else if (status.state == "DISABLED") {
            row.status_label->override_color(m_color_red);
        } else {
            row.status_label->override_color(m_color_orange);
        }

        std::string substate_text;
        if (status.is_homed) {
            substate_text = "[HOMED]";
        } else {
            substate_text = "[NOT HOMED]";
        }
        if (status.is_busy) {
            substate_text += " [BUSY]";
        }
        row.substate_label->set_text(substate_text);

        if (row.tool_label) {
            row.tool_label->set_text(status.tool_type.empty() ? " - No Instrument" : " - " + status.tool_type);
        }

        // Ensure row is visible since messages have arrived
        if (!row.container->get_visible()) {
            row.container->set_visible(true);
            row.container->show_all();
        }
    }

    void handle_arm_tool_type(const std::string& arm_name, const std::string& tool_type) {
        auto& status = m_arms_status[arm_name];
        status.tool_type = tool_type;

        auto& row = m_arm_rows[arm_name];
        if (row.tool_label) {
            row.tool_label->set_text(tool_type.empty() ? " - No Instrument" : " - " + tool_type);
        }
    }

    void handle_teleop_enabled_event(bool enabled) {
        m_teleop_master_enabled = enabled;
        m_teleop_enable_status->set_text(enabled ? "ENABLED" : "DISABLED");
        if (enabled) {
            m_teleop_enable_status->override_color(m_color_green);
            m_teleop_enable_btn->override_color(m_color_green);
        } else {
            m_teleop_enable_status->override_color(m_color_red);
            m_teleop_enable_btn->override_color(m_color_red);
        }
    }

    void on_master_enable_clicked() {
        std_msgs::msg::Bool msg;
        msg.data = !m_teleop_master_enabled;
        m_teleop_enable_pub->publish(msg);
    }

    void on_dvrk_power_clicked() {
        auto msg = std_msgs::msg::Empty();
        if (m_dvrk_powered_on) {
            m_dvrk_power_off_pub->publish(msg);
        } else {
            m_dvrk_home_pub->publish(msg);
        }
    }

    void handle_teleop_event(const std::string& name, bool selected) {
        m_active_teleops_states[name] = selected;

        auto it = m_teleop_rows.find(name);
        if (it == m_teleop_rows.end()) {
            TeleopRow row;
            row.container = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 10));
            row.container->set_margin_bottom(4);
            row.container->set_margin_left(28); // Indent to show hierarchy under master button

            row.action_button = Gtk::manage(new Gtk::Button(selected ? "-" : "+"));
            row.action_button->get_style_context()->add_class("reset-btn");
            row.action_button->set_valign(Gtk::ALIGN_CENTER);
            row.action_button->set_margin_right(12);

            Gtk::Box* text_box = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 2));

            // First line: Name + Selected/Not Selected status
            Gtk::Box* first_line = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 6));
            row.name_label = Gtk::manage(new Gtk::Label(name, Gtk::ALIGN_START));
            row.name_label->get_style_context()->add_class("arm-name");
            row.status_label = Gtk::manage(new Gtk::Label(selected ? "SELECTED" : "UNSELECTED", Gtk::ALIGN_START));
            row.status_label->get_style_context()->add_class("arm-status");
            first_line->pack_start(*row.name_label, Gtk::PACK_SHRINK);
            first_line->pack_start(*row.status_label, Gtk::PACK_SHRINK);

            // Second line: Current State
            Gtk::Box* second_line = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 6));
            row.current_state_label = Gtk::manage(new Gtk::Label("OFFLINE", Gtk::ALIGN_START));
            row.current_state_label->get_style_context()->add_class("arm-tool");
            second_line->pack_start(*row.current_state_label, Gtk::PACK_SHRINK);

            text_box->pack_start(*first_line, Gtk::PACK_SHRINK);
            text_box->pack_start(*second_line, Gtk::PACK_SHRINK);

            if (selected) {
                row.status_label->override_color(m_color_green);
            } else {
                row.status_label->override_color(m_color_orange);
            }

            row.container->pack_start(*row.action_button, Gtk::PACK_SHRINK, 0);
            row.container->pack_start(*text_box, Gtk::PACK_EXPAND_WIDGET, 0);

            // Connect button click
            row.action_button->signal_clicked().connect([this, name]() {
                on_teleop_button_clicked(name);
            });

            m_teleops_box.pack_start(*row.container, Gtk::PACK_SHRINK, 0);
            row.container->show_all();

            m_teleop_rows[name] = row;

            // Subscribe dynamically to its current state topic
            auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            std::string state_topic = "/" + name + "/current_state";
            m_teleop_state_subs[name] = m_node->create_subscription<std_msgs::msg::String>(
                state_topic, qos, [this, name](const std_msgs::msg::String::SharedPtr msg) {
                    handle_teleop_current_state(name, msg->data);
                });
        } else {
            auto& row = it->second;
            row.action_button->set_label(selected ? "-" : "+");
            row.status_label->set_text(selected ? "SELECTED" : "UNSELECTED");
            if (selected) {
                row.status_label->override_color(m_color_green);
            } else {
                row.status_label->override_color(m_color_orange);
            }
        }

        // Reorder teleop list to keep ECM at the bottom
        reorder_teleop_list();
    }

    void reorder_teleop_list() {
        auto children = m_teleops_box.get_children();
        if (children.size() <= 2) return; // Master + 0 or 1 item, no reordering needed

        std::vector<Gtk::Widget*> non_ecm;
        std::vector<Gtk::Widget*> ecm;

        for (size_t i = 1; i < children.size(); ++i) {
            Gtk::Widget* w = children[i];
            std::string name = "";
            for (const auto& pair : m_teleop_rows) {
                if (pair.second.container == w) {
                    name = pair.first;
                    break;
                }
            }
            if (name.empty()) continue;

            if (name.find("ECM") != std::string::npos) {
                ecm.push_back(w);
            } else {
                non_ecm.push_back(w);
            }
        }

        int pos = 1;
        for (auto* w : non_ecm) {
            m_teleops_box.reorder_child(*w, pos++);
        }
        for (auto* w : ecm) {
            m_teleops_box.reorder_child(*w, pos++);
        }
    }

    void handle_teleop_current_state(const std::string& name, const std::string& state) {
        auto it = m_teleop_rows.find(name);
        if (it != m_teleop_rows.end()) {
            auto& row = it->second;
            row.current_state_label->set_text(state);
            // Color code current state label: green for ACTIVE/following, red for DISABLED, orange otherwise
            if (state == "ENABLED" || state == "following") {
                row.current_state_label->override_color(m_color_green);
            } else if (state == "DISABLED") {
                row.current_state_label->override_color(m_color_red);
            } else {
                row.current_state_label->override_color(m_color_orange);
            }
        }
    }

    void on_teleop_button_clicked(const std::string& name) {
        bool current = m_active_teleops_states[name];
        std_msgs::msg::String msg;
        msg.data = name;
        if (current) {
            m_teleop_unselect_pub->publish(msg);
        } else {
            m_teleop_select_pub->publish(msg);
        }
    }

    void update_scale_ui(double scale) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - m_scale_last_change_time).count() < 1000) {
            // Ignore incoming updates while user is interacting
            return;
        }

        if (scale < 0.2) {
            scale = 0.2;
        }
        scale = std::round(scale * 10.0) / 10.0;
        m_last_scale_request = scale;

        m_scale_connection.block();
        m_scale_slider->set_value(scale);
        m_scale_connection.unblock();
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << scale;
        m_scale_value_label->set_text(ss.str());
    }

    void on_scale_slider_changed() {
        m_scale_last_change_time = std::chrono::steady_clock::now();
        double val = m_scale_slider->get_value();
        if (val < 0.2) {
            val = 0.2;
            m_scale_connection.block();
            m_scale_slider->set_value(0.2);
            m_scale_connection.unblock();
        }
        // Round to nearest 0.1
        val = std::round(val * 10.0) / 10.0;

        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << val;
        m_scale_value_label->set_text(ss.str());

        if (val == m_last_scale_request) {
            return;
        }

        m_last_scale_request = val;
        std_msgs::msg::Float64 msg;
        msg.data = val;
        m_scale_pub->publish(msg);
    }

    void update_volume_ui(double volume) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - m_volume_last_change_time).count() < 1000) {
            // Ignore incoming updates while user is interacting
            return;
        }

        m_volume_connection.block();
        m_volume_slider->set_value(volume);
        m_volume_connection.unblock();
        volume = std::round(volume * 20.0) / 20.0;
        m_last_volume_request = volume;
        
        std::stringstream ss;
        ss << std::round(volume * 100.0) << "%";
        m_volume_value_label->set_text(ss.str());
    }

    void on_volume_slider_changed() {
        m_volume_last_change_time = std::chrono::steady_clock::now();
        double val = m_volume_slider->get_value();
        // Round to nearest 0.05
        val = std::round(val * 20.0) / 20.0;

        std::stringstream ss;
        ss << std::round(val * 100.0) << "%";
        m_volume_value_label->set_text(ss.str());

        if (val == m_last_volume_request) {
            return;
        }

        m_last_volume_request = val;
        std_msgs::msg::Float64 msg;
        msg.data = val;
        m_volume_pub->publish(msg);
    }

    Gdk::RGBA menu_item_text_color() const {
        Gdk::RGBA color;
        color.set(m_dark_mode ? "#ffffff" : "#1a202c");
        return color;
    }

    void apply_menu_item_text_color(Gtk::MenuItem& item) {
        const auto color = menu_item_text_color();
        item.override_color(color, Gtk::STATE_FLAG_NORMAL);
        if (auto* child = item.get_child()) {
            child->override_color(color, Gtk::STATE_FLAG_NORMAL);
        }
    }

    Gtk::MenuItem* create_menu_item(const std::string& label) {
        auto* item = Gtk::manage(new Gtk::MenuItem());
        auto* item_label = Gtk::manage(new Gtk::Label(label, Gtk::ALIGN_START));
        item->add(*item_label);
        apply_menu_item_text_color(*item);
        return item;
    }

    void append_video_source_items(Gtk::Menu& menu) {
        for (const auto& source : m_video_sources) {
            auto* item = create_menu_item(source.label);
            item->signal_activate().connect([this, source]() {
                open_video_source(source.socket_path);
            });
            menu.append(*item);
        }
    }

    void on_wrench_clicked() {
        // Clear menu first to rebuild dynamically
        auto children = m_wrench_menu.get_children();
        for (auto* child : children) {
            m_wrench_menu.remove(*child);
        }

        // 1. Add "Use video" submenu
        if (m_video_sources.size() > 1) {
            auto* use_video_item = create_menu_item("Use video");
            auto* video_sources_menu = Gtk::manage(new Gtk::Menu());
            append_video_source_items(*video_sources_menu);
            use_video_item->set_submenu(*video_sources_menu);
            apply_menu_item_text_color(*use_video_item);
            m_wrench_menu.append(*use_video_item);
        }

        // 2. Add "Send to" submenu
        auto* send_to_item = create_menu_item("Send to");
        auto* monitors_menu = Gtk::manage(new Gtk::Menu());
        send_to_item->set_submenu(*monitors_menu);
        apply_menu_item_text_color(*send_to_item);

        auto display = Gdk::Display::get_default();
        if (display) {
            int n_monitors = display->get_n_monitors();
            for (int i = 0; i < n_monitors; ++i) {
                auto monitor = display->get_monitor(i);
                std::string model = monitor->get_model();
                std::string label = model.empty() ? "Monitor " + std::to_string(i + 1) : model;
                
                auto* monitor_item = create_menu_item(label);
                monitor_item->signal_activate().connect([this, i]() {
                    send_to_monitor(i);
                });
                monitors_menu->append(*monitor_item);
            }
        }
        m_wrench_menu.append(*send_to_item);

        // 3. Add "Fullscreen" toggle item
        bool is_fullscreen = is_window_fullscreen();
        auto* fullscreen_item = create_menu_item(is_fullscreen ? "Exit Fullscreen" : "Fullscreen");
        fullscreen_item->signal_activate().connect([this, is_fullscreen]() {
            m_window_fullscreen = !is_fullscreen;
            apply_window_display_settings();
            save_persisted_display_settings();
        });
        m_wrench_menu.append(*fullscreen_item);

        // 4. Add "Dark Mode" toggle item
        auto* dark_mode_item = create_menu_item(m_dark_mode ? "Light Mode" : "Dark Mode");
        dark_mode_item->signal_activate().connect(sigc::mem_fun(*this, &ControlPanelWindow::toggle_dark_mode));
        m_wrench_menu.append(*dark_mode_item);

        // 5. Add "Quit" item
        auto* quit_item = create_menu_item("Quit");
        quit_item->signal_activate().connect([this]() {
            close();
        });
        m_wrench_menu.append(*quit_item);

        m_wrench_menu.show_all();
        m_wrench_menu.popup_at_widget(m_wrench_btn, Gdk::GRAVITY_SOUTH_EAST, Gdk::GRAVITY_NORTH_WEST, nullptr);
    }

    void send_to_monitor(int monitor_idx) {
        auto display = Gdk::Display::get_default();
        if (display && monitor_idx >= 0 && monitor_idx < display->get_n_monitors()) {
            m_window_monitor_index = monitor_idx;
            apply_window_display_settings();
            save_persisted_display_settings();
        }
    }

    bool is_window_fullscreen() const {
        auto gdk_window = get_window();
        return gdk_window &&
               ((gdk_window->get_state() & Gdk::WINDOW_STATE_FULLSCREEN) != 0);
    }

    std::string settings_file_path() const {
        std::string safe_name = m_settings_name.empty() ? "dvrk_display" : m_settings_name;
        for (char& ch : safe_name) {
            const unsigned char uch = static_cast<unsigned char>(ch);
            if (!std::isalnum(uch) && ch != '-' && ch != '_') {
                ch = '_';
            }
        }

        return Glib::build_filename(
            Glib::build_filename(Glib::get_user_config_dir(), "dvrk_display"),
            safe_name + "_control_panel_gui.ini");
    }

    void load_persisted_display_settings() {
        m_have_persisted_display_settings = false;
        const std::string path = settings_file_path();
        if (!std::filesystem::exists(path)) {
            return;
        }

        try {
            Glib::KeyFile key_file;
            key_file.load_from_file(path);
            m_window_monitor_index = key_file.get_integer("window", "monitor");
            m_window_fullscreen = key_file.get_boolean("window", "fullscreen");
            m_have_persisted_display_settings = true;
        } catch (const Glib::Error& error) {
            std::cerr << "Warning: unable to load display settings from '" << path
                      << "': " << error.what() << std::endl;
        }
    }

    void save_persisted_display_settings() {
        try {
            const std::string path = settings_file_path();
            std::filesystem::create_directories(
                std::filesystem::path(path).parent_path());

            Glib::KeyFile key_file;
            key_file.set_string("viewer", "name", m_settings_name);
            key_file.set_integer("window", "monitor", m_window_monitor_index);
            key_file.set_boolean("window", "fullscreen", m_window_fullscreen);
            Glib::file_set_contents(path, key_file.to_data());
        } catch (const Glib::Error& error) {
            std::cerr << "Warning: unable to save display settings: "
                      << error.what() << std::endl;
        } catch (const std::exception& error) {
            std::cerr << "Warning: unable to save display settings: "
                      << error.what() << std::endl;
        }
    }

    void apply_window_display_settings() {
        auto display = Gdk::Display::get_default();
        if (!display || display->get_n_monitors() <= 0) {
            return;
        }

        const int target_monitor = std::max(
            0, std::min(m_window_monitor_index, display->get_n_monitors() - 1));
        m_window_monitor_index = target_monitor;

        auto monitor = display->get_monitor(target_monitor);
        if (!monitor) {
            return;
        }

        Gdk::Rectangle rect;
        monitor->get_geometry(rect);
        const int monitor_x = rect.get_x();
        const int monitor_y = rect.get_y();
        const int monitor_width = rect.get_width();
        const int monitor_height = rect.get_height();

        unmaximize();
        unfullscreen();
        move(monitor_x, monitor_y);
        present();

        if (m_window_fullscreen) {
            Gtk::Window* window = this;
            Glib::signal_idle().connect_once([window, monitor_x, monitor_y,
                                               monitor_width, monitor_height]() {
                if (window) {
                    window->move(monitor_x, monitor_y);
                    window->resize(monitor_width, monitor_height);
                    window->fullscreen();
                    window->present();
                }
            });
        }
    }

    bool poll_window_monitor() {
        const int monitor_idx = monitor_index_for_window();
        if (monitor_idx >= 0 && monitor_idx != m_window_monitor_index) {
            m_window_monitor_index = monitor_idx;
            save_persisted_display_settings();
        }
        return true;
    }

    int monitor_index_for_window() {
        auto display = Gdk::Display::get_default();
        if (!display) {
            return -1;
        }
        auto gdk_window = get_window();
        if (!gdk_window) {
            return -1;
        }
        auto monitor = display->get_monitor_at_window(gdk_window);
        if (!monitor) {
            return -1;
        }
        const int n = display->get_n_monitors();
        for (int i = 0; i < n; ++i) {
            if (display->get_monitor(i) == monitor) {
                return i;
            }
        }
        return -1;
    }

    void toggle_dark_mode() {
        m_dark_mode = !m_dark_mode;
        if (m_dark_mode) {
            m_color_green.set("#2ec4b6");
            m_color_red.set("#e71d36");
            m_color_orange.set("#ff9f1c");
        } else {
            m_color_green.set("#00875a");
            m_color_red.set("#de350b");
            m_color_orange.set("#ff8c00");
        }

        // Trigger CSS reload
        int width, height;
        get_size(width, height);
        update_dynamic_css(width, height);

        // Re-apply operating state color coding for all visible arm rows
        for (const auto& pair : m_arms_status) {
            const auto& arm_name = pair.first;
            const auto& status = pair.second;
            if (status.active) {
                auto& row = m_arm_rows[arm_name];
                if (status.state == "ENABLED") {
                    row.status_label->override_color(m_color_green);
                } else if (status.state == "DISABLED") {
                    row.status_label->override_color(m_color_red);
                } else {
                    row.status_label->override_color(m_color_orange);
                }
            }
        }

        // Re-apply color coding for master dVRK power status
        update_dvrk_power_state(true);

        // Re-apply color coding for master teleop enable status
        if (m_teleop_master_enabled) {
            m_teleop_enable_status->override_color(m_color_green);
            m_teleop_enable_btn->override_color(m_color_green);
        } else {
            m_teleop_enable_status->override_color(m_color_red);
            m_teleop_enable_btn->override_color(m_color_red);
        }

        // Re-apply color coding for active teleops
        for (const auto& pair : m_active_teleops_states) {
            const auto& name = pair.first;
            bool selected = pair.second;
            auto it = m_teleop_rows.find(name);
            if (it != m_teleop_rows.end()) {
                auto& row = it->second;
                if (selected) {
                    row.status_label->override_color(m_color_green);
                } else {
                    row.status_label->override_color(m_color_orange);
                }
            }
        }
    }

    void trigger_arm_reset(const std::string& name) {
        // Enqueue reset sequence
        ResetState reset;
        reset.arm_name = name;
        reset.stage = ResetState::IDLE;
        reset.last_command_time = std::chrono::steady_clock::now();

        m_active_resets[name] = reset;
        m_reset_timer_active = true;
    }

    void publish_arm_state_command(const std::string& arm_name, const std::string& state_cmd) {
        auto msg = crtk_msgs::msg::StringStamped();
        msg.string = state_cmd;
        msg.header.stamp = m_node->get_clock()->now();
        m_arm_pubs[arm_name]->publish(msg);
    }

    bool on_ui_tick() {
        if (!m_reset_timer_active) {
            return true;
        }

        auto now = std::chrono::steady_clock::now();
        std::vector<std::string> completed_resets;

        for (auto& pair : m_active_resets) {
            auto& reset = pair.second;
            const auto& status = m_arms_status[reset.arm_name];

            switch (reset.stage) {
                case ResetState::IDLE:
                    std::cout << "[Reset] Disabling arm " << reset.arm_name << std::endl;
                    publish_arm_state_command(reset.arm_name, "disable");
                    reset.stage = ResetState::WAITING_FOR_DISABLE;
                    reset.last_command_time = now;
                    break;

                case ResetState::WAITING_FOR_DISABLE:
                    if (status.state == "DISABLED") {
                        std::cout << "[Reset] Arm disabled. Enabling arm " << reset.arm_name << std::endl;
                        publish_arm_state_command(reset.arm_name, "enable");
                        reset.stage = ResetState::WAITING_FOR_ENABLE;
                        reset.last_command_time = now;
                    } else if (std::chrono::duration_cast<std::chrono::seconds>(now - reset.last_command_time).count() > 3) {
                        std::cout << "[Reset] Timeout waiting for disable. Re-enabling anyways." << std::endl;
                        publish_arm_state_command(reset.arm_name, "enable");
                        reset.stage = ResetState::WAITING_FOR_ENABLE;
                        reset.last_command_time = now;
                    }
                    break;

                case ResetState::WAITING_FOR_ENABLE:
                    if (status.state == "ENABLED") {
                        std::cout << "[Reset] Arm enabled. Homing arm " << reset.arm_name << std::endl;
                        publish_arm_state_command(reset.arm_name, "home");
                        reset.stage = ResetState::WAITING_FOR_HOME;
                        reset.last_command_time = now;
                    } else if (std::chrono::duration_cast<std::chrono::seconds>(now - reset.last_command_time).count() > 3) {
                        std::cout << "[Reset] Timeout waiting for enable. Sequence aborted." << std::endl;
                        reset.stage = ResetState::FINISHED;
                    }
                    break;

                case ResetState::WAITING_FOR_HOME:
                    if (status.state == "ENABLED" && status.is_homed && !status.is_busy) {
                        std::cout << "[Reset] Arm " << reset.arm_name << " home complete." << std::endl;
                        reset.stage = ResetState::FINISHED;
                    } else if (std::chrono::duration_cast<std::chrono::seconds>(now - reset.last_command_time).count() > 15) {
                        std::cout << "[Reset] Timeout waiting for home. Sequence aborted." << std::endl;
                        reset.stage = ResetState::FINISHED;
                    }
                    break;

                case ResetState::FINISHED:
                    completed_resets.push_back(reset.arm_name);
                    break;
            }
        }

        // Clean up completed resets
        for (const auto& arm : completed_resets) {
            m_active_resets.erase(arm);
        }

        if (m_active_resets.empty()) {
            m_reset_timer_active = false;
        }

        return true;
    }

    // ROS2 structures
    std::shared_ptr<rclcpp::Node> m_node;
    std::string m_settings_name;
    std::string m_console_name;
    std::vector<VideoSource> m_video_sources;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_teleop_selected_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_teleop_unselected_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_teleop_enabled_sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_teleop_enable_pub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_scale_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_scale_pub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_volume_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_volume_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_teleop_select_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_teleop_unselect_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_dvrk_home_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_dvrk_power_off_pub;

    std::unordered_map<std::string, rclcpp::Subscription<crtk_msgs::msg::OperatingState>::SharedPtr> m_arm_subs;
    std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> m_tool_subs;
    std::unordered_map<std::string, rclcpp::Publisher<crtk_msgs::msg::StringStamped>::SharedPtr> m_arm_pubs;

    // Dynamically populated teleop current state subscribers
    std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> m_teleop_state_subs;

    // UI Widgets
    Gtk::Paned m_main_paned;
    Gtk::Box m_left_pane;
    Gtk::Box m_right_pane;

    Gtk::Frame m_arms_frame;
    Gtk::Box m_arms_box;

    Gtk::Frame m_teleops_frame;
    Gtk::Box m_teleops_box;
    Gtk::Button* m_teleop_enable_btn;
    Gtk::Label* m_teleop_enable_status;
    Gtk::Button* m_dvrk_power_btn;
    Gtk::Label* m_dvrk_power_status;

    Gtk::Frame m_system_frame;
    Gtk::Scale* m_scale_slider;
    Gtk::Label* m_scale_value_label;
    Gtk::Scale* m_volume_slider;
    Gtk::Label* m_volume_value_label;
    Gtk::Button* m_wrench_btn;
    Gtk::Menu m_wrench_menu;

    Gtk::Box m_video_area{Gtk::ORIENTATION_VERTICAL};

    // UI State & Mappings
    struct ArmRow {
        Gtk::Box* container;
        Gtk::Label* name_label;
        Gtk::Label* status_label;
        Gtk::Label* substate_label;
        Gtk::Label* tool_label; // nullptr for non-PSMs
        Gtk::Button* reset_button;
    };
    std::unordered_map<std::string, ArmRow> m_arm_rows;
    std::unordered_map<std::string, ArmStatus> m_arms_status;

    struct TeleopRow {
        Gtk::Box* container;
        Gtk::Button* action_button; // "+" or "-"
        Gtk::Label* name_label;
        Gtk::Label* status_label;
        Gtk::Label* current_state_label;
    };
    std::unordered_map<std::string, TeleopRow> m_teleop_rows;
    std::unordered_map<std::string, bool> m_active_teleops_states;

    GstElement* m_current_video_pipeline = nullptr;
    Gtk::Widget* m_current_video_widget = nullptr;
    std::string m_current_video_path;

    // Asynchronous resets
    std::unordered_map<std::string, ResetState> m_active_resets;
    bool m_reset_timer_active;

    // Slider signal connections for loopback control
    sigc::connection m_ui_tick_connection;
    sigc::connection m_monitor_poll_connection;
    sigc::connection m_scale_connection;
    sigc::connection m_volume_connection;
    std::chrono::steady_clock::time_point m_scale_last_change_time;
    std::chrono::steady_clock::time_point m_volume_last_change_time;
    double m_last_scale_request = 0.5;
    double m_last_volume_request = 0.5;

    // Teleop state
    bool m_teleop_master_enabled;

    // dVRK power state
    bool m_dvrk_powered_on;

    // Styles & CSS
    Glib::RefPtr<Gtk::CssProvider> m_css_provider;

    // Theme options
    bool m_dark_mode;
    int m_window_monitor_index = 0;
    bool m_window_fullscreen = false;
    bool m_have_persisted_display_settings = false;

    // Colors
    Gdk::RGBA m_color_green;
    Gdk::RGBA m_color_red;
    Gdk::RGBA m_color_orange;
};

// Main Entry Point
int main(int argc, char* argv[]) {
    // The control panel has no text-entry widgets.  Avoid routing GTK
    // input through IBus, which can still trigger GNOME's on-screen keyboard
    // even when the accessibility screen-keyboard setting is disabled.
    setenv("GTK_IM_MODULE", "gtk-im-context-simple", 1);

    CommandLineOptions options;
    if (!parse_arguments(argc, argv, options)) {
        print_usage(argv[0]);
        return 1;
    }

    ControlPanelConfig config;
    if (!options.config_file.empty() &&
        !load_control_panel_config(options.config_file, config)) {
        return 1;
    }

    if (options.console_override) {
        config.console = options.console_name;
    }
    for (const auto& source : options.video_sources) {
        if (!source.empty()) {
            config.video_sources.push_back(source);
        }
    }

    std::vector<VideoSource> video_sources;
    video_sources.reserve(config.video_sources.size());
    for (const auto& source : config.video_sources) {
        video_sources.push_back(make_video_source(config.name, source));
    }

    // Initialize GStreamer
    gst_init(&argc, &argv);

    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dvrk_control_panel_node");

    // Initialize GTK Application
    auto app = Gtk::Application::create("org.dvrk.display.control_panel", Gio::APPLICATION_NON_UNIQUE);

    // Create the Window
    ControlPanelWindow window(node, config.name, config.console, video_sources);

    // Wire up ROS2 spin with GLib main loop (every 20ms)
    guint ros_spin_source = g_timeout_add(20, [](gpointer data) -> gboolean {
        auto* n = static_cast<rclcpp::Node*>(data);
        if (rclcpp::ok()) {
            rclcpp::spin_some(n->get_node_base_interface());
            return TRUE;
        }
        return FALSE;
    }, node.get());

    // Gracefully handle termination signals to quit Gtk main loop
    g_unix_signal_add(SIGINT, [](gpointer data) -> gboolean {
        auto* application = static_cast<Gtk::Application*>(data);
        if (application) {
            application->quit();
        }
        return FALSE;
    }, app.get());

    g_unix_signal_add(SIGTERM, [](gpointer data) -> gboolean {
        auto* application = static_cast<Gtk::Application*>(data);
        if (application) {
            application->quit();
        }
        return FALSE;
    }, app.get());

    // Run GTK application
    int status = app->run(window);
    g_source_remove(ros_spin_source);

    // Shutdown ROS2
    rclcpp::shutdown();

    return status;
}

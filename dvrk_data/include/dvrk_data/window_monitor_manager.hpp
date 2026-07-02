#ifndef DVRK_DATA_WINDOW_MONITOR_MANAGER_HPP
#define DVRK_DATA_WINDOW_MONITOR_MANAGER_HPP

#include <gtkmm.h>
#include <glibmm/keyfile.h>
#include <glibmm/fileutils.h>
#include <glibmm/miscutils.h>
#include <glibmm/main.h>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cctype>

namespace sv {

class WindowMonitorManager {
public:
    WindowMonitorManager(Gtk::Window& window, const std::string& app_name, const std::string& suffix)
        : m_window(window), m_app_name(app_name), m_suffix(suffix) {}

    Gtk::Window& get_window() { return m_window; }

    int get_monitor_index() const { return m_monitor_index; }
    void set_monitor_index(int idx) { m_monitor_index = idx; }

    bool get_fullscreen() const { return m_fullscreen; }
    void set_fullscreen(bool fs) { m_fullscreen = fs; }

    std::string settings_file_path() const {
        std::string safe_name = m_app_name.empty() ? "dvrk_display" : m_app_name;
        for (char& ch : safe_name) {
            const unsigned char uch = static_cast<unsigned char>(ch);
            if (!std::isalnum(uch) && ch != '-' && ch != '_') {
                ch = '_';
            }
        }
        return Glib::build_filename(
            Glib::build_filename(Glib::get_user_config_dir(), "dvrk_display"),
            safe_name + "_" + m_suffix + "_gui.ini");
    }

    void load_settings(const std::string& group = "window") {
        const std::string path = settings_file_path();
        if (!std::filesystem::exists(path)) {
            return;
        }
        try {
            Glib::KeyFile key_file;
            key_file.load_from_file(path);
            if (key_file.has_group(group)) {
                if (key_file.has_key(group, "monitor")) {
                    m_monitor_index = key_file.get_integer(group, "monitor");
                }
                if (key_file.has_key(group, "fullscreen")) {
                    m_fullscreen = key_file.get_boolean(group, "fullscreen");
                }
            }
        } catch (const Glib::Error& error) {
            std::cerr << "Warning: unable to load display settings from '" << path
                      << "': " << error.what() << std::endl;
        }
    }

    void save_settings(const std::string& group = "window") {
        try {
            const std::string path = settings_file_path();
            std::filesystem::create_directories(std::filesystem::path(path).parent_path());

            Glib::KeyFile key_file;
            if (std::filesystem::exists(path)) {
                key_file.load_from_file(path);
            }
            key_file.set_string("viewer", "name", m_app_name);
            key_file.set_integer(group, "monitor", m_monitor_index);
            key_file.set_boolean(group, "fullscreen", m_fullscreen);
            Glib::file_set_contents(path, key_file.to_data());
        } catch (const Glib::Error& error) {
            std::cerr << "Warning: unable to save display settings: "
                      << error.what() << std::endl;
        } catch (const std::exception& error) {
            std::cerr << "Warning: unable to save display settings: "
                      << error.what() << std::endl;
        }
    }

    static std::vector<std::string> get_monitor_labels() {
        std::vector<std::string> labels;
        auto display = Gdk::Display::get_default();
        if (!display) {
            return labels;
        }
        const int count = display->get_n_monitors();
        for (int i = 0; i < count; ++i) {
            auto monitor = display->get_monitor(i);
            if (!monitor) continue;
            Gdk::Rectangle geometry;
            monitor->get_geometry(geometry);
            std::string model = monitor->get_model();
            std::string manufacturer = monitor->get_manufacturer();
            std::ostringstream ss;
            ss << i << ": ";
            if (!manufacturer.empty()) {
                ss << manufacturer;
                if (!model.empty()) ss << " ";
            }
            if (!model.empty()) {
                ss << model << " ";
            }
            ss << geometry.get_width() << "x" << geometry.get_height();
            labels.push_back(ss.str());
        }
        return labels;
    }

    int get_current_monitor_index() const {
        auto display = Gdk::Display::get_default();
        if (!display) return -1;
        auto gdk_window = m_window.get_window();
        if (!gdk_window) return -1;
        auto monitor = display->get_monitor_at_window(gdk_window);
        if (!monitor) return -1;
        const int count = display->get_n_monitors();
        for (int i = 0; i < count; ++i) {
            if (display->get_monitor(i) == monitor) {
                return i;
            }
        }
        return -1;
    }

    void apply_display_settings(int offset = 0) {
        auto display = Gdk::Display::get_default();
        if (!display || display->get_n_monitors() <= 0) {
            return;
        }

        const int target_monitor = std::max(
            0, std::min(m_monitor_index, display->get_n_monitors() - 1));
        m_monitor_index = target_monitor;

        auto monitor = display->get_monitor(target_monitor);
        if (!monitor) {
            return;
        }

        Gdk::Rectangle rect;
        monitor->get_geometry(rect);
        const int monitor_x = rect.get_x() + offset;
        const int monitor_y = rect.get_y() + offset;
        const int monitor_width = rect.get_width();
        const int monitor_height = rect.get_height();

        m_window.unmaximize();
        m_window.unfullscreen();
        m_window.move(monitor_x, monitor_y);
        m_window.present();

        if (m_fullscreen) {
            auto screen = m_window.get_screen();
            Gtk::Window* win_ptr = &m_window;
            Glib::signal_idle().connect_once([win_ptr, screen, target_monitor, monitor_x, monitor_y, monitor_width, monitor_height]() {
                if (win_ptr) {
                    win_ptr->move(monitor_x, monitor_y);
                    win_ptr->resize(monitor_width, monitor_height);
                    if (screen) {
                        win_ptr->fullscreen_on_monitor(screen, target_monitor);
                    } else {
                        win_ptr->fullscreen();
                    }
                    win_ptr->present();
                }
            });
        }
    }

    void send_to_monitor(int monitor_idx, int offset = 0, const std::string& group = "window") {
        auto display = Gdk::Display::get_default();
        if (display && monitor_idx >= 0 && monitor_idx < display->get_n_monitors()) {
            m_monitor_index = monitor_idx;
            apply_display_settings(offset);
            save_settings(group);
        }
    }

    bool poll_window_monitor(const std::string& group = "window") {
        const int idx = get_current_monitor_index();
        if (idx >= 0 && idx != m_monitor_index) {
            m_monitor_index = idx;
            save_settings(group);
        }
        return true;
    }

    void populate_send_to_menu(Gtk::Menu& menu, int offset = 0, const std::string& group = "window",
                               std::function<Gtk::MenuItem*(const std::string&)> item_creator = nullptr,
                               std::function<void()> on_activated = nullptr) {
        auto display = Gdk::Display::get_default();
        if (!display) return;
        int n_monitors = display->get_n_monitors();
        for (int i = 0; i < n_monitors; ++i) {
            auto monitor = display->get_monitor(i);
            if (!monitor) continue;
            Gdk::Rectangle geometry;
            monitor->get_geometry(geometry);
            std::string model = monitor->get_model();
            std::string manufacturer = monitor->get_manufacturer();
            std::ostringstream ss;
            ss << i << ": ";
            if (!manufacturer.empty()) {
                ss << manufacturer;
                if (!model.empty()) ss << " ";
            }
            if (!model.empty()) {
                ss << model << " ";
            }
            ss << geometry.get_width() << "x" << geometry.get_height();
            std::string label = ss.str();
            
            Gtk::MenuItem* monitor_item = nullptr;
            if (item_creator) {
                monitor_item = item_creator(label);
            } else {
                monitor_item = Gtk::manage(new Gtk::MenuItem(label));
            }
            monitor_item->signal_activate().connect([this, i, offset, group, on_activated]() {
                send_to_monitor(i, offset, group);
                if (on_activated) {
                    on_activated();
                }
            });
            menu.append(*monitor_item);
        }
    }

private:
    Gtk::Window& m_window;
    std::string m_app_name;
    std::string m_suffix;
    int m_monitor_index = 0;
    bool m_fullscreen = false;
};

} // namespace sv

#endif // DVRK_DATA_WINDOW_MONITOR_MANAGER_HPP

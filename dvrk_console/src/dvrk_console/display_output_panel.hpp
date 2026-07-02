#ifndef DVRK_DISPLAY_OUTPUT_PANEL_HPP
#define DVRK_DISPLAY_OUTPUT_PANEL_HPP

#include <gdk/gdkkeysyms.h>
#include <glibmm/fileutils.h>
#include <glibmm/keyfile.h>
#include <glibmm/main.h>
#include <glibmm/miscutils.h>
#include <gtkmm.h>

#include <cctype>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <dvrk_data/config.hpp>
#include <dvrk_data/window_monitor_manager.hpp>

namespace sv {

class DisplayOutputPanel {
public:
  struct SinkDescriptor {
    std::string sink_name;
    std::string label;
    std::string window_title;
    int default_width = 640;
    int default_height = 480;
    GtkWidget *gtk_widget = nullptr;
  };

  using SinkSettingsMap = std::unordered_map<std::string, std::pair<int, bool>>;

  DisplayOutputPanel(const AppConfig &cfg, std::string settings_suffix,
                     std::function<void()> quit_cb)
      : m_cfg(cfg), m_settings_suffix(std::move(settings_suffix)),
        m_quit_cb(std::move(quit_cb)), m_sink_controls_box(Gtk::ORIENTATION_VERTICAL) {
    m_frame.set_label("Display Outputs");
    m_sink_controls_box.set_spacing(6);
    m_frame.add(m_sink_controls_box);
    load_persisted_settings();
  }

  Gtk::Frame &widget() { return m_frame; }

  void rebuild(const std::vector<SinkDescriptor> &descriptors) {
    auto previous_settings = capture_live_settings();
    for (const auto &[sink_name, settings] : m_persisted_sink_settings) {
      previous_settings.emplace(sink_name, settings);
    }

    clear_ui();
    m_sink_controls.clear();
    update_monitor_options();

    for (const auto &descriptor : descriptors) {
      if (!descriptor.gtk_widget) {
        continue;
      }

      auto win = std::make_unique<Gtk::Window>();
      win->add_events(Gdk::KEY_PRESS_MASK);
      win->signal_key_press_event().connect(
          [this](GdkEventKey *event) {
            if ((event->state & GDK_CONTROL_MASK) &&
                (event->keyval == GDK_KEY_q)) {
              on_quit_clicked();
              return true;
            }
            return false;
          },
          false);
      win->set_title(descriptor.window_title);
      win->set_default_size(descriptor.default_width, descriptor.default_height);

      Gtk::Widget *mm_widget = Glib::wrap(descriptor.gtk_widget);
      win->add(*mm_widget);
      win->show_all();

      SinkDisplayControl control;
      control.sink_name = descriptor.sink_name;
      control.label_text = descriptor.label;
      control.window = std::move(win);
      append_sink_control_row(control, previous_settings);
      m_sink_controls.push_back(std::move(control));

      g_object_unref(descriptor.gtk_widget);
    }

    if (!m_sink_controls.empty()) {
      m_monitor_poll_connection = Glib::signal_timeout().connect(
          sigc::mem_fun(*this, &DisplayOutputPanel::poll_window_monitors), 1000);
    }

    m_frame.set_visible(!m_sink_controls.empty());
    m_frame.show_all_children();
    apply_all_display_settings();
  }

private:
  struct SinkDisplayControl {
    std::string sink_name;
    std::string label_text;
    Gtk::Box *row = nullptr;
    Gtk::Label *label = nullptr;
    Gtk::ComboBoxText *monitor_combo = nullptr;
    sigc::connection monitor_combo_connection;
    Gtk::CheckButton *fullscreen_btn = nullptr;
    std::unique_ptr<Gtk::Window> window;
    std::unique_ptr<WindowMonitorManager> monitor_manager;
  };

  static int clip_int(const int value, const int min_value,
                      const int max_value) {
    return std::max(min_value, std::min(max_value, value));
  }

  SinkSettingsMap capture_live_settings() const {
    SinkSettingsMap settings;
    for (const auto &control : m_sink_controls) {
      int monitor_index = 0;
      bool fullscreen = false;
      if (control.monitor_combo) {
        const int active = control.monitor_combo->get_active_row_number();
        if (active >= 0) {
          monitor_index = active;
        }
      }
      if (control.fullscreen_btn) {
        fullscreen = control.fullscreen_btn->get_active();
      }
      settings[control.sink_name] = {monitor_index, fullscreen};
    }
    return settings;
  }

  std::string settings_file_path() const {
    std::string safe_name = m_cfg.name.empty() ? "dvrk_display" : m_cfg.name;
    for (char &ch : safe_name) {
      const unsigned char uch = static_cast<unsigned char>(ch);
      if (!std::isalnum(uch) && ch != '-' && ch != '_') {
        ch = '_';
      }
    }
    return Glib::build_filename(
        Glib::build_filename(Glib::get_user_config_dir(), "dvrk_display"),
        safe_name + "_" + m_settings_suffix + "_gui.ini");
  }

  void load_persisted_settings() {
    // Handled individually by WindowMonitorManager per sink
  }

  void save_persisted_settings() {
    for (auto &control : m_sink_controls) {
      if (control.monitor_combo && control.monitor_manager) {
        const int active = control.monitor_combo->get_active_row_number();
        control.monitor_manager->set_monitor_index(active >= 0 ? active : 0);
        control.monitor_manager->set_fullscreen(control.fullscreen_btn && control.fullscreen_btn->get_active());
        control.monitor_manager->save_settings("sink:" + control.sink_name);
      }
    }
  }

  void clear_ui() {
    m_monitor_poll_connection.disconnect();
    auto children = m_sink_controls_box.get_children();
    for (auto *child : children) {
      m_sink_controls_box.remove(*child);
    }
  }

  void update_monitor_options() {
    m_monitor_labels.clear();
    auto display = Gdk::Display::get_default();
    if (!display) {
      return;
    }

    const int monitor_count = display->get_n_monitors();
    for (int i = 0; i < monitor_count; ++i) {
      auto monitor = display->get_monitor(i);
      if (!monitor) {
        continue;
      }

      Gdk::Rectangle geometry;
      monitor->get_geometry(geometry);

      std::ostringstream label;
      label << i << ": ";

      const auto manufacturer = monitor->get_manufacturer();
      const auto model = monitor->get_model();
      if (!manufacturer.empty()) {
        label << manufacturer;
        if (!model.empty()) {
          label << " ";
        }
      }
      if (!model.empty()) {
        label << model << " ";
      }
      label << geometry.get_width() << "x" << geometry.get_height();
      m_monitor_labels.push_back(label.str());
    }
  }

  void append_sink_control_row(SinkDisplayControl &control,
                               const SinkSettingsMap &previous_settings) {
    const std::string sink_name = control.sink_name;
    control.row = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 8));
    control.label = Gtk::manage(new Gtk::Label(control.label_text));
    control.label->set_halign(Gtk::ALIGN_START);
    control.label->set_size_request(90, -1);

    control.monitor_combo = Gtk::manage(new Gtk::ComboBoxText());
    if (m_monitor_labels.empty()) {
      control.monitor_combo->append("0: Default");
    } else {
      for (const auto &label : m_monitor_labels) {
        control.monitor_combo->append(label);
      }
    }

    control.monitor_manager = std::make_unique<WindowMonitorManager>(
        *control.window, m_cfg.name, m_settings_suffix);
    control.monitor_manager->load_settings("sink:" + control.sink_name);

    int initial_monitor = control.monitor_manager->get_monitor_index();
    bool initial_fullscreen = control.monitor_manager->get_fullscreen();

    if (!m_monitor_labels.empty()) {
      initial_monitor = clip_int(initial_monitor, 0,
                                 static_cast<int>(m_monitor_labels.size()) - 1);
    } else {
      initial_monitor = 0;
    }

    control.monitor_combo->set_active(initial_monitor);
    control.fullscreen_btn = Gtk::manage(new Gtk::CheckButton("Fullscreen"));
    control.fullscreen_btn->set_active(initial_fullscreen);

    control.monitor_combo_connection =
        control.monitor_combo->signal_changed().connect([this, sink_name]() {
          apply_display_selection(sink_name);
          save_persisted_settings();
        });
    control.fullscreen_btn->signal_toggled().connect([this, sink_name]() {
      apply_display_selection(sink_name);
      save_persisted_settings();
    });

    control.row->pack_start(*control.label, Gtk::PACK_SHRINK);
    control.row->pack_start(*control.monitor_combo, Gtk::PACK_EXPAND_WIDGET);
    control.row->pack_start(*control.fullscreen_btn, Gtk::PACK_SHRINK);
    m_sink_controls_box.pack_start(*control.row, Gtk::PACK_SHRINK);
  }

  void apply_all_display_settings() {
    for (auto &control : m_sink_controls) {
      apply_display_selection(control);
    }
  }

  void apply_display_selection(const std::string &sink_name) {
    for (auto &control : m_sink_controls) {
      if (control.sink_name == sink_name) {
        apply_display_selection(control);
        return;
      }
    }
  }

  void apply_display_selection(SinkDisplayControl &control) {
    if (!control.monitor_combo || !control.monitor_manager) {
      return;
    }
    const int active = control.monitor_combo->get_active_row_number();
    control.monitor_manager->set_monitor_index(active >= 0 ? active : 0);
    control.monitor_manager->set_fullscreen(control.fullscreen_btn && control.fullscreen_btn->get_active());
    
    const int offset = 40 * sink_display_order(control.sink_name);
    control.monitor_manager->apply_display_settings(offset);
  }

  // Polls window positions every second and syncs the monitor combo when a
  // window is dragged to a different display. Returns true to keep running.
  bool poll_window_monitors() {
    for (auto &control : m_sink_controls) {
      if (!control.monitor_combo || !control.window || !control.monitor_manager) {
        continue;
      }
      control.monitor_manager->poll_window_monitor("sink:" + control.sink_name);
      const int idx = control.monitor_manager->get_monitor_index();
      if (idx >= 0 && idx != control.monitor_combo->get_active_row_number()) {
        control.monitor_combo_connection.block();
        control.monitor_combo->set_active(idx);
        control.monitor_combo_connection.unblock();
      }
    }
    return true;
  }

  int sink_display_order(const std::string &sink_name) const {
    for (std::size_t i = 0; i < m_sink_controls.size(); ++i) {
      if (m_sink_controls[i].sink_name == sink_name) {
        return static_cast<int>(i);
      }
    }
    return 0;
  }

  void on_quit_clicked() {
    if (m_quit_cb) {
      m_quit_cb();
    }
  }

  const AppConfig &m_cfg;
  std::string m_settings_suffix;
  std::function<void()> m_quit_cb;
  Gtk::Frame m_frame;
  Gtk::Box m_sink_controls_box;
  std::vector<std::string> m_monitor_labels;
  std::vector<SinkDisplayControl> m_sink_controls;
  SinkSettingsMap m_persisted_sink_settings;
  sigc::connection m_monitor_poll_connection;
};

}  // namespace sv

#endif

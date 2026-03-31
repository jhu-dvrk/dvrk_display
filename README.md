# dvrk_display

Starter ROS 2 package for a dVRK-specific stereo viewer application.

Current shell contents:
- `stereo` C++ node
- JSON config loader using JsonCpp
- GStreamer dependency wiring in CMake
- `stereo_calibration` helper script
- sample config under `share/dvrk_display.json`

The current node loads one or more JSON config files and builds a side-by-side GStreamer pipeline from root-level stereo settings.

Optional root-level field `ros_image_publishers` controls ROS image republishing from the viewer. It must be a list of values from:
- `left`
- `right`
- `stereo`

For each enabled value, the node publishes via `image_transport::CameraPublisher` on:
- `<name>/left/image_raw` + `<name>/left/camera_info`
- `<name>/right/image_raw` + `<name>/right/camera_info`
- `<name>/stereo/image_raw` + `<name>/stereo/camera_info`

The viewer now subscribes to:
- `/<console>/camera` (`sensor_msgs/msg/Joy`)
- `/<console>/clutch` (`sensor_msgs/msg/Joy`)

Set `"dvrk_console_namespace"` in the JSON root to choose the namespace prefix. If omitted (or empty), the default is `"console"`.

Both subscriptions use transient-local durability (latched/persistent behavior), and the decoded status is rendered as the same overlay on both left and right channels.

`unixfd_socket_path` behavior:
- omitted: unixfd publishing enabled with default path `/tmp/dvrk_display_<user>.sock`
- empty string: unixfd publishing disabled
- non-empty path: publish to that exact socket path

Config type tag is `sv::dvrk_display_config@1.0.0`.

At startup, the node always logs unixfd status. If unixfd is requested but the runtime doesn't provide a compatible FD upload path, unixfd is automatically disabled with a warning and the viewer continues.

# dvrk_stereo_viewer

Starter ROS 2 package for a dVRK-specific stereo viewer application.

Current shell contents:
- `dvrk_stereo_viewer` C++ node
- JSON config loader using JsonCpp
- GStreamer dependency wiring in CMake
- `stereo_configurator` helper script
- sample config under `share/stereo_viewer.json`

The current node loads one or more JSON config files and builds a side-by-side GStreamer pipeline from the `stereo` settings.

The viewer now subscribes to:
- `/<console>/camera` (`sensor_msgs/msg/Joy`)
- `/<console>/clutch` (`sensor_msgs/msg/Joy`)

Set `"console"` in the JSON root to choose the namespace prefix. If omitted (or empty), the default is `"console"`.

Both subscriptions use transient-local durability (latched/persistent behavior), and the decoded status is rendered as the same overlay block on both left and right channels.

`stereo.unixfd_socket_path` behavior:
- omitted: unixfd publishing enabled with default path `/tmp/dvrk_stereo_viewer_<user>.sock`
- empty string: unixfd publishing disabled
- non-empty path: publish to that exact socket path

At startup, the node always logs unixfd status. If unixfd is requested but the runtime doesn't provide a compatible FD upload path, unixfd is automatically disabled with a warning and the viewer continues.

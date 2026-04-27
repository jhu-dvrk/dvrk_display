import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def check_socket_path(context, *args, **kwargs):
    viewer_name = LaunchConfiguration("viewer_name").perform(context)
    stream = LaunchConfiguration("stream").perform(context)
    user = os.environ.get("USER", "")
    socket_path = f"/tmp/{viewer_name}_{stream}_{user}.sock"

    if not os.path.exists(socket_path):
        raise RuntimeError(
            f"Socket path '{socket_path}' does not exist. "
            f"Please ensure dvrk_display is running and configured with a unixfdsink for stream '{stream}'."
        )


def generate_launch_description():
    viewer_name_arg = DeclareLaunchArgument(
        "viewer_name",
        default_value="dvrk_display",
        description="Viewer name used by stereo for implicit unixfd socket naming",
    )
    stream_arg = DeclareLaunchArgument(
        "stream",
        default_value="stereo",
        description="Stream name suffix used in the implicit unixfd socket path",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=[LaunchConfiguration("viewer_name"), "/", LaunchConfiguration("stream"), "/camera"],
        description=(
            "ROS namespace for gscam. image_raw, image_raw/compressed, "
            "and camera_info are all published under this namespace."
        ),
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value=[LaunchConfiguration("viewer_name"), "_", LaunchConfiguration("stream"), "_frame"],
        description="frame_id stamped on published images",
    )
    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value=[LaunchConfiguration("viewer_name"), "_", LaunchConfiguration("stream")],
        description="camera_name stored in camera_info metadata",
    )

    gscam_config = PythonExpression([
        "'unixfdsrc socket-path=/tmp/' + '",
        LaunchConfiguration("viewer_name"),
        "' + '_' + '",
        LaunchConfiguration("stream"),
        "' + '_' + '",
        EnvironmentVariable("USER"),
        "' + '.sock ! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream ! videoconvert'",
    ])

    gscam_node = Node(
        package="gscam",
        executable="gscam_node",
        name=LaunchConfiguration("camera_name"),
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "gscam_config": gscam_config,
                "camera_name": LaunchConfiguration("camera_name"),
                "frame_id": LaunchConfiguration("frame_id"),
                "use_gst_timestamps": False,
                "sync_sink": False,
            }
        ],
    )

    return LaunchDescription([
        viewer_name_arg,
        stream_arg,
        namespace_arg,
        frame_id_arg,
        camera_name_arg,
        OpaqueFunction(function=check_socket_path),
        gscam_node,
    ])
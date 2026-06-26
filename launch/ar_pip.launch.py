import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Evaluate the ar_source parameter
    ar_source_val = LaunchConfiguration('ar_source').perform(context)
    
    if ar_source_val == "gstreamer":
        script_name = "gstreamer_ar.py"
        config_name = "test_gstreamer_ar_pip.json"
    else: # default "opencv"
        script_name = "opencv_ar.py"
        config_name = "test_opencv_ar_pip.json"

    # Retrieve path to the config file installed in the share directory
    pkg_share = get_package_share_directory('dvrk_display')
    config_file_path = os.path.join(pkg_share, config_name)

    # Launch chosen AR script to create the unixfd sockets
    ar_node = Node(
        package="dvrk_display",
        executable=script_name,
        name="ar_overlay",
        output="screen",
        arguments=["--width", "640", "--height", "480"]
    )

    # Launch stereo viewer with the config file path
    stereo_node = Node(
        package="dvrk_display",
        executable="stereo",
        name="stereo_viewer",
        output="screen",
        arguments=["-c", config_file_path]
    )

    # Delay the dvrk_display stereo viewer by 2.0 seconds to give the AR script
    # enough time to initialize and bind the unix socket servers.
    delayed_stereo = TimerAction(
        period=2.0,
        actions=[stereo_node]
    )

    return [
        ar_node,
        delayed_stereo
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ar_source',
            default_value='opencv',
            choices=['opencv', 'gstreamer'],
            description='AR overlay source to use (opencv or gstreamer)'
        ),
        OpaqueFunction(function=launch_setup)
    ])

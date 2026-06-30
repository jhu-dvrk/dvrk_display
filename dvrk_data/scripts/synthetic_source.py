#!/usr/bin/env python3

import sys
import os
import time
import threading
import argparse
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

try:
    import cairo
except ImportError:
    print("Error: python3-cairo is required. Install it with: sudo apt install python3-cairo")
    sys.exit(1)

class SyntheticSource(Node):
    def __init__(self, left_socket, right_socket, correlation_topic, width, height, fps):
        super().__init__('dvrk_synthetic_source')
        self.pub = self.create_publisher(Header, correlation_topic, 10)
        self.counter = 0
        self.counter_lock = threading.Lock()
        self.left_socket = left_socket
        self.right_socket = right_socket
        
        Gst.init(None)
        
        # Remove existing socket files to avoid bind errors
        for socket_path in [left_socket, right_socket]:
            if os.path.exists(socket_path):
                try:
                    os.unlink(socket_path)
                except OSError:
                    pass

        # Pipeline: videotestsrc -> cairooverlay -> unixfdsink
        pipeline_str = (
            f"videotestsrc pattern=ball is-live=true ! video/x-raw,width={width},height={height},framerate={fps}/1 ! videoconvert ! video/x-raw,format=BGRA ! "
            "cairooverlay name=overlay_l ! videoconvert ! video/x-raw,format=I420 ! "
            "queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! "
            f"unixfdsink socket-path={left_socket} sync=true async=false "
            
            f"videotestsrc pattern=smpte is-live=true ! video/x-raw,width={width},height={height},framerate={fps}/1 ! videoconvert ! video/x-raw,format=BGRA ! "
            "cairooverlay name=overlay_r ! videoconvert ! video/x-raw,format=I420 ! "
            "queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! "
            f"unixfdsink socket-path={right_socket} sync=true async=false"
        )

        
        self.get_logger().info("Initializing GStreamer pipeline...")
        self.pipeline = Gst.parse_launch(pipeline_str)
        if not self.pipeline:
            self.get_logger().error("Failed to parse pipeline")
            sys.exit(1)
            
        self.overlay_l = self.pipeline.get_by_name("overlay_l")
        self.overlay_r = self.pipeline.get_by_name("overlay_r")
        
        self.overlay_l.connect("draw", self.on_draw, "LEFT")
        self.overlay_r.connect("draw", self.on_draw, "RIGHT")
        
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self.on_message)

        # Register Unix signal handler for SIGINT and SIGTERM to stop cleanly
        import signal
        GLib.unix_signal_add(GLib.PRIORITY_HIGH, signal.SIGINT, self.sigint_handler)
        GLib.unix_signal_add(GLib.PRIORITY_HIGH, signal.SIGTERM, self.sigint_handler)

    def sigint_handler(self):
        self.get_logger().info("Interrupted by user (SIGINT/SIGTERM)")
        self.stop()
        return GLib.SOURCE_REMOVE

    def on_draw(self, overlay, context, timestamp, duration, eye):
        with self.counter_lock:
            # We increment the counter only on one eye to keep them in sync per frame
            if eye == "LEFT":
                self.counter += 1
            local_counter = self.counter
            
        curr_time = self.get_clock().now()
        
        # Draw on frame using Cairo
        context.select_font_face("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
        
        # Background box for readability
        context.set_source_rgba(0, 0, 0, 0.6)
        context.rectangle(40, 60, 450, 160)
        context.fill()
        
        context.set_source_rgb(0.0, 1.0, 0.0) # Green for ID
        context.set_font_size(40)
        msg = f"CORRELATION ID: {local_counter}"
        context.move_to(50, 100)
        context.show_text(msg)
        
        context.set_source_rgb(1, 1, 1) # White for TS
        context.set_font_size(20)
        ts_msg = f"ROS TS: {curr_time.nanoseconds}"
        context.move_to(50, 140)
        context.show_text(ts_msg)
        
        context.set_source_rgb(0.7, 0.7, 1.0) # Light blue for Eye
        context.set_font_size(30)
        context.move_to(50, 190)
        context.show_text(f"STREAM: {eye} EYE")

        # Publish ROS message 
        # We publish the correlation data to ROS so 'record' can capture it.
        if rclpy.ok() and eye == "LEFT":
            header = Header()
            header.stamp = curr_time.to_msg()
            header.frame_id = f"test_source:{local_counter}"
            try:
                self.pub.publish(header)
            except Exception:
                pass

    def on_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS:
            self.get_logger().info("End-of-stream")
            GLib.idle_add(self.stop)
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f"GStreamer Error: {err.message}")
            if debug:
                self.get_logger().debug(f"Debug: {debug}")
            GLib.idle_add(self.stop)

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)
        for socket_path in [self.left_socket, self.right_socket]:
            if os.path.exists(socket_path):
                try:
                    os.unlink(socket_path)
                except OSError:
                    pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        os._exit(0)

    def run(self):
        self.get_logger().info(f"Starting pipeline. Streams will be available at {self.left_socket} and {self.right_socket}")
        self.pipeline.set_state(Gst.State.PLAYING)
        
        # Use a separate thread for rclpy.spin to allow GStreamer to run its own loop
        def safe_spin():
            try:
                rclpy.spin(self)
            except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
                pass
            except Exception:
                pass

        spin_thread = threading.Thread(target=safe_spin, daemon=True)
        spin_thread.start()
        
        loop = GLib.MainLoop()
        loop.run()

def main():
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    parser = argparse.ArgumentParser(description="Synthetic unixfd stereo source with a ROS correlation topic")
    parser.add_argument("--left-socket", default="/tmp/dvrk_test_l")
    parser.add_argument("--right-socket", default="/tmp/dvrk_test_r")
    parser.add_argument("--correlation-topic", default="/dvrk_test/correlation")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    args, _ = parser.parse_known_args()
    
    rclpy.init()
    node = SyntheticSource(
        args.left_socket,
        args.right_socket,
        args.correlation_topic,
        args.width,
        args.height,
        args.fps,
    )
    node.run()

if __name__ == '__main__':
    main()

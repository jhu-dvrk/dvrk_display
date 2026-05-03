#!/usr/bin/env python3

import sys
import os
import time
import threading
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
    def __init__(self):
        super().__init__('dvrk_synthetic_source')
        self.pub = self.create_publisher(Header, '/dvrk_test/correlation', 10)
        self.counter = 0
        self.counter_lock = threading.Lock()
        
        Gst.init(None)
        
        # Pipeline: videotestsrc -> cairooverlay -> shmsink
        # We use a reasonably large shm-size to avoid drops.
        # Format I420 640x480 is ~460KB per frame.
        pipeline_str = (
            "videotestsrc pattern=ball is-live=true ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=BGRA ! "
            "cairooverlay name=overlay_l ! videoconvert ! video/x-raw,format=I420 ! shmsink socket-path=/tmp/dvrk_test_l shm-size=10000000 wait-for-connection=false sync=true "
            "videotestsrc pattern=smpte is-live=true ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=BGRA ! "
            "cairooverlay name=overlay_r ! videoconvert ! video/x-raw,format=I420 ! shmsink socket-path=/tmp/dvrk_test_r shm-size=10000000 wait-for-connection=false sync=true"
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
        if eye == "LEFT":
            header = Header()
            header.stamp = curr_time.to_msg()
            header.frame_id = f"test_source:{local_counter}"
            self.pub.publish(header)

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
        rclpy.shutdown()
        sys.exit(0)

    def run(self):
        self.get_logger().info("Starting pipeline. Streams will be available at /tmp/dvrk_test_l and /tmp/dvrk_test_r")
        self.pipeline.set_state(Gst.State.PLAYING)
        
        # Use a separate thread for rclpy.spin to allow GStreamer to run its own loop
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()
        
        try:
            loop = GLib.MainLoop()
            loop.run()
        except KeyboardInterrupt:
            self.get_logger().info("Interrupted by user")
        finally:
            self.pipeline.set_state(Gst.State.NULL)

def main():
    rclpy.init()
    node = SyntheticSource()
    node.run()

if __name__ == '__main__':
    main()

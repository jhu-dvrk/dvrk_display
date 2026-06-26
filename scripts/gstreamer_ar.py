#!/usr/bin/env python3

import sys
import os
import time
import math
import argparse
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

try:
    import cairo
except ImportError:
    print("Error: python3-cairo is required. Install it with: sudo apt install python3-cairo")
    sys.exit(1)

class SimpleAR:
    def __init__(self, left_socket, right_socket, width, height, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self.left_socket = left_socket
        self.right_socket = right_socket

        Gst.init(None)

        # Remove existing socket files to avoid bind errors
        for socket_path in [left_socket, right_socket]:
            if os.path.exists(socket_path):
                print(f"Removing existing socket: {socket_path}")
                try:
                    os.unlink(socket_path)
                except OSError as e:
                    print(f"Error removing {socket_path}: {e}")

        # Pipeline: videotestsrc (transparent) -> cairooverlay -> unixfdsink
        pipeline_str = (
            f"videotestsrc pattern=solid-color foreground-color=0 background-color=0 is-live=true ! "
            f"video/x-raw,format=BGRA,width={width},height={height},framerate={fps}/1 ! "
            f"cairooverlay name=overlay_l ! "
            f"queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! "
            f"unixfdsink socket-path={left_socket} sync=true async=false "
            
            f"videotestsrc pattern=solid-color foreground-color=0 background-color=0 is-live=true ! "
            f"video/x-raw,format=BGRA,width={width},height={height},framerate={fps}/1 ! "
            f"cairooverlay name=overlay_r ! "
            f"queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! "
            f"unixfdsink socket-path={right_socket} sync=true async=false"
        )

        print("Initializing GStreamer pipeline...")
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except GLib.Error as e:
            print(f"Error: Failed to parse GStreamer pipeline: {e}")
            sys.exit(1)

        self.overlay_l = self.pipeline.get_by_name("overlay_l")
        self.overlay_r = self.pipeline.get_by_name("overlay_r")

        if not self.overlay_l or not self.overlay_r:
            print("Error: Failed to find overlay elements in pipeline.")
            sys.exit(1)

        self.overlay_l.connect("draw", self.on_draw, "LEFT")
        self.overlay_r.connect("draw", self.on_draw, "RIGHT")

        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self.on_message)

    def on_draw(self, overlay, context, timestamp, duration, eye):
        t = time.time()
        
        # Center of screen
        cx = self.width / 2.0
        cy = self.height / 2.0
        
        # Move in a figure-8 pattern
        x = cx + (self.width * 0.3) * math.sin(t * 1.5)
        y = cy + (self.height * 0.2) * math.sin(t * 3.0)
        
        # Add stereo disparity (depth animation)
        disparity = 8.0 * math.sin(t * 0.5)
        if eye == "LEFT":
            x += disparity
        else:
            x -= disparity

        # Draw a simple crosshair (2 perpendicular lines)
        size = 20.0
        context.set_line_width(2.0)
        context.set_source_rgba(0.0, 1.0, 0.0, 1.0) # Solid green
        
        # Horizontal line
        context.move_to(x - size, y)
        context.line_to(x + size, y)
        
        # Vertical line
        context.move_to(x, y - size)
        context.line_to(x, y + size)
        
        context.stroke()

    def on_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS:
            print("End-of-stream reached.")
            GLib.idle_add(self.stop)
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"GStreamer Error: {err.message}")
            if debug:
                print(f"Debug: {debug}")
            GLib.idle_add(self.stop)

    def start(self):
        print("Streaming simple Cairo AR feeds...")
        print(f"AR Left socket: {self.left_socket}")
        print(f"AR Right socket: {self.right_socket}")
        print("Press Ctrl+C to stop.")
        
        self.pipeline.set_state(Gst.State.PLAYING)
        try:
            self.loop = GLib.MainLoop()
            self.loop.run()
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.stop()

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)
        for socket_path in [self.left_socket, self.right_socket]:
            if os.path.exists(socket_path):
                try:
                    os.unlink(socket_path)
                except OSError:
                    pass
        print("Sockets cleaned up.")

def main():
    left_socket = "/tmp/dvrk_display_left_ar.sock"
    right_socket = "/tmp/dvrk_display_right_ar.sock"

    parser = argparse.ArgumentParser(description="Cairo-based 3D AR Overlay source for dVRK console")
    parser.add_argument("-w", "--width", type=int, default=640, help="Width of frame (default: 640)")
    parser.add_argument("-H", "--height", type=int, default=480, help="Height of frame (default: 480)")

    args, _ = parser.parse_known_args()

    app = SimpleAR(left_socket, right_socket, args.width, args.height)
    app.start()

if __name__ == '__main__':
    main()

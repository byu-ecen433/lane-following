#!/usr/bin/env python3
"""
ECEN 433 - Lab 4
================

PROVIDED TOOL - read it, but do not edit it.

Hover your cursor over the image and read the HSV value of the pixel under it.
This is how you pick colour thresholds for your own robot in Part V, instead of
guessing and rebuilding.

    dts devel run -X -R <robot> -L inspect_hsv      # your robot's camera
    dts devel run -X -L inspect_hsv                 # the sample images

It subscribes to the same relative topic your detector does, so it works
against either source with no change - the namespace decides which. Point it
somewhere else with:

    roslaunch camera_sim hsv_inspector.launch image_topic:=some/other/topic

The readout under the image gives you H, S, V and R, G, B for the pixel you are
pointing at. Note the coordinates are in the FULL camera frame; your detector
sees a resized and cropped version, but colour does not change when you resize.
"""

import numpy as np
import cv2
import rospy
import matplotlib
import matplotlib.pyplot as plt
from sensor_msgs.msg import CompressedImage

DEFAULT_IMAGE_TOPIC = "camera_node/image/compressed"
DEFAULT_REFRESH_S = 1.0


class HSVInspector:
    def __init__(self):
        rospy.init_node("hsv_inspector")

        self.image_topic = rospy.get_param("~image_topic", DEFAULT_IMAGE_TOPIC)
        self.refresh_s = float(rospy.get_param("~refresh_seconds", DEFAULT_REFRESH_S))

        self.bgr = None
        self.hsv = None

        rospy.Subscriber(self.image_topic, CompressedImage, self.image_cb,
                         queue_size=1, buff_size=2 ** 24)

        rospy.loginfo("hsv_inspector reading %s", rospy.resolve_name(self.image_topic))

    def image_cb(self, msg):
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            self.bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr("could not decode image: %s", e)

    def _format_coord(self, x, y):
        """What matplotlib prints under the cursor."""
        if self.hsv is None:
            return ""
        col, row = int(x + 0.5), int(y + 0.5)
        h, w = self.hsv.shape[:2]
        if not (0 <= col < w and 0 <= row < h):
            return f"x={col} y={row}"
        hh, ss, vv = self.hsv[row, col]
        b, g, r = self.bgr[row, col]
        return (f"x={col} y={row}   "
                f"H={hh:3d}  S={ss:3d}  V={vv:3d}      "
                f"(R={r:3d} G={g:3d} B={b:3d})")

    def run(self):
        rospy.loginfo("waiting for the first image...")
        while self.bgr is None and not rospy.is_shutdown():
            rospy.sleep(0.2)
        if rospy.is_shutdown():
            return

        fig, ax = plt.subplots(figsize=(9, 7))
        ax.set_title("hover to read HSV - close the window or Ctrl-C to quit")
        ax.format_coord = self._format_coord
        handle = ax.imshow(cv2.cvtColor(self.bgr, cv2.COLOR_BGR2RGB))
        ax.axis("off")
        fig.tight_layout()
        plt.show(block=False)

        while not rospy.is_shutdown() and plt.fignum_exists(fig.number):
            self.hsv = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2HSV)
            handle.set_data(cv2.cvtColor(self.bgr, cv2.COLOR_BGR2RGB))
            fig.canvas.draw_idle()
            # Keeps the window responsive to the mouse between frames.
            plt.pause(self.refresh_s)


if __name__ == "__main__":
    try:
        HSVInspector().run()
    except rospy.ROSInterruptException:
        pass

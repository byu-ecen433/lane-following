#!/usr/bin/env python3
"""
ECEN 433 - Lab 4
================

PROVIDED NODE - read it, but do not edit it.

This node pretends to be your Duckiebot's camera. It replays the saved images
in `sample_images/` onto the exact topic the real camera node publishes on:

    camera_node/image/compressed   (sensor_msgs/CompressedImage)

"""

import os

import rospy
import rospkg
from sensor_msgs.msg import CompressedImage

IMAGE_TOPIC = "camera_node/image/compressed"

PUBLISH_RATE_HZ = 20.0      # roughly what the real camera manages
SECONDS_PER_IMAGE = 5.0     # how long to sit on each sample image


class CameraSimNode:
    def __init__(self):
        rospy.init_node("camera_node")

        self.rate_hz = rospy.get_param("~publish_rate", PUBLISH_RATE_HZ)
        self.seconds_per_image = rospy.get_param("~seconds_per_image", SECONDS_PER_IMAGE)
        self.frame_id = rospy.get_param("~frame_id", "camera_optical_frame")

        self.pub_image = rospy.Publisher(IMAGE_TOPIC, CompressedImage, queue_size=1)

        self.jpegs = self._load_jpegs()

        rospy.loginfo(
            "camera_sim replaying %d images on %s at %.1f Hz",
            len(self.jpegs), rospy.resolve_name(IMAGE_TOPIC), self.rate_hz,
        )

    def _load_jpegs(self):
        """Read every sample image off disk as raw JPEG bytes, in name order."""
        pkg_path = rospkg.RosPack().get_path("camera_sim")
        image_dir = os.path.join(pkg_path, "sample_images")

        names = sorted(n for n in os.listdir(image_dir) if n.lower().endswith(".jpg"))
        if not names:
            rospy.logfatal("No .jpg files found in %s", image_dir)
            raise rospy.ROSInitException("no sample images")

        jpegs = []
        for name in names:
            with open(os.path.join(image_dir, name), "rb") as f:
                jpegs.append(f.read())
        return jpegs

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        index = 0
        held_since = rospy.get_time()

        while not rospy.is_shutdown():
            # Advance to the next image once we have shown this one long enough.
            if rospy.get_time() - held_since >= self.seconds_per_image:
                index = (index + 1) % len(self.jpegs)
                held_since = rospy.get_time()

            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.format = "jpeg"
            msg.data = self.jpegs[index]
            self.pub_image.publish(msg)

            rate.sleep()


if __name__ == "__main__":
    try:
        CameraSimNode().run()
    except rospy.ROSInterruptException:
        pass

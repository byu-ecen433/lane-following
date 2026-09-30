#!/usr/bin/env python3
"""
ECEN 433 - Lab 4
================

Find the white, yellow and red lane markings in a camera image and publish them
as line segments for the rest of the Duckietown stack to use.

Debug and tune your parameters in the notebook: lane_detection_activity.ipynb.
"""

import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_srvs.srv import SetBool, SetBoolResponse
from duckietown_msgs.msg import Segment, SegmentList

# BGR colours used for drawing debug images.
DRAW_COLORS = {
    "WHITE": (255, 255, 255),
    "YELLOW": (0, 255, 255),
    "RED": (0, 0, 255),
}

# Segment.WHITE == 0, Segment.YELLOW == 1, Segment.RED == 2. Looked up by name
# so a typo raises here instead of silently mislabelling every segment.
SEGMENT_COLOR_IDS = {
    "WHITE": Segment.WHITE,
    "YELLOW": Segment.YELLOW,
    "RED": Segment.RED,
}

class ColorRange:
    """One colour's HSV threshold, as one range or multiple ranges.

    Red can require two. Hue is an angle: OpenCV packs it into 0..179, so red sits at
    both ends of that scale at once and no single low/high pair can catch it.
    Every other colour on the road is one contiguous band and needs one range.

    The parsing below is done for you. It reads whichever shape your param file
    uses - a single `low`/`high` pair, or numbered `low_1`/`high_1`,
    `low_2`/`high_2` pairs - and leaves you a list of (low, high) tuples in
    `self.bounds`.
    """

    def __init__(self, name, config, hough_defaults=None):
        self.name = name

        # Hough parameters default for every colour, and each colour may
        # override any of them. 
        h = dict(hough_defaults or {})
        h.update(config.get("hough", {}))
        self.hough_threshold = int(h.get("threshold", 4))
        self.hough_min_line_length = int(h.get("min_line_length", 3))
        self.hough_max_line_gap = int(h.get("max_line_gap", 3))

        self.bounds = []
        if "low" in config:
            self.bounds.append((np.array(config["low"], dtype=np.uint8),
                                np.array(config["high"], dtype=np.uint8)))
        i = 1
        while f"low_{i}" in config:
            self.bounds.append((np.array(config[f"low_{i}"], dtype=np.uint8),
                                np.array(config[f"high_{i}"], dtype=np.uint8)))
            i += 1
        if not self.bounds:
            raise ValueError(f"colour {name} has no low/high or low_1/high_1 pair")

    def mask(self, hsv):
        """The binary mask of every pixel of this colour."""
        # TODO (Part II): cv2.inRange for each (low, high) in self.bounds,
        # combined with cv2.bitwise_or. Notebook section 3.
        raise NotImplementedError("ColorRange.mask")


class LaneDetectorNode:
    def __init__(self):
        rospy.init_node("lane_detector_node")
        self.get_params(event=None)
        self.bridge = CvBridge()

        # TODO: segment publisher
        self.pub_segments = None

        # Debug views, rendered only when something is subscribed.
        self.pub_cropped = rospy.Publisher("~image_cropped", Image, queue_size=1)
        self.pub_edges = rospy.Publisher("~image_edges", Image, queue_size=1)
        self.pub_masks = {
            name: rospy.Publisher(f"~mask_{name.lower()}", Image, queue_size=1)
            for name in self.colors
        }
        self.pub_lines = {
            name: rospy.Publisher(f"~image_lines_{name.lower()}", Image, queue_size=1)
            for name in self.colors
        }
        self.pub_lines_all = rospy.Publisher("~image_lines_all", Image, queue_size=1)

        # TODO: subscribe to image
        self.sub_image = None

        # We replaced Duckietown's line detector, so we answer its switch
        # service in its place.
        rospy.Service("~switch", SetBool, self._switch)
        if rospy.get_param("~fake_lane_filter_switch", False):
            rospy.Service("lane_filter_node/switch", SetBool, self._switch)

        rospy.loginfo("lane_detector_node subscribed to %s, detecting %s",
                      rospy.resolve_name(self.image_topic),
                      ", ".join(sorted(self.colors)))

    def get_params(self, event):

        self.image_topic = rospy.get_param("~image_topic", "camera_node/image/compressed")
        img_size = rospy.get_param("~img_size", [160, 120])
        self.top_cutoff = float(rospy.get_param("~top_cutoff", 0.5))
        hough_defaults = rospy.get_param("~hough", {})
        self.colors = {name: ColorRange(name, cfg, hough_defaults)
                        for name, cfg in rospy.get_param("~colors").items()}
        kernel_size = int(rospy.get_param("~dilation_kernel_size", 3))
        self.erode_iterations = int(rospy.get_param("~erode_iterations", 1))
        self.dilate_iterations = int(rospy.get_param("~dilate_iterations", 1))
        self.canny_thresholds = rospy.get_param("~canny_thresholds", [80, 200])
        self.canny_aperture_size = int(rospy.get_param("~canny_aperture_size", 3))
        self.normal_probe_px = int(rospy.get_param("~normal_probe_px", 2))


        # Computed variables from parameters.
        self.img_w, self.img_h = int(img_size[0]), int(img_size[1])
        if not 0.0 <= self.top_cutoff < 1.0:
            raise ValueError(
                f"~top_cutoff is a fraction of image height and must be in "
                f"[0.0, 1.0), got {self.top_cutoff}.")
        self.cutoff_rows = int(self.top_cutoff * self.img_h)
        self.kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    

    # ----------------------------------------------------------------------
    # Parts II and III
    # ----------------------------------------------------------------------

    def image_cb(self, msg):
        """Called for every frame. Keep it fast - this runs 20-30 times a second."""
        try:
            bgr = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("could not decode image: %s", e)
            return

        # TODO (Part II): resize to (self.img_w, self.img_h), THEN slice off
        # the top self.cutoff_rows rows. That order matters!
        cropped = None

        # TODO (Part II): BGR to HSV. Notebook section 2.
        hsv = None

        # TODO (Part II): a cleaned mask per colour. self.colors maps name ->
        # ColorRange; erode then dilate with self.kernel and the iteration
        # counts from the param file. The dilation is what makes the mask reach
        # the Canny edges on its boundary. Notebook sections 3 and 4.
        masks = {}

        # TODO (Part III): Canny once over `cropped`, not once per colour.
        # Notebook section 5.
        edges = None

        detections = {}
        for name, mask in masks.items():
            # TODO (Part III): cv2.bitwise_and the mask with the edges, then
            # cv2.HoughLinesP on the result, using THIS colour's parameters -
            # self.colors[name].hough_threshold, .hough_min_line_length and
            # .hough_max_line_gap. minLineLength and maxLineGap must both be
            # > 0. Handle a None return, and reshape to (-1, 4) - this OpenCV
            # returns Nx4. Notebook sections 6 and 7.
            lines = np.zeros((0, 4), dtype=int)

            normals = self._orient(lines, mask)
            detections[name] = (lines, normals)

        if cropped is None:
            rospy.logwarn_once(
                "image_cb is not doing anything yet - frames are arriving but "
                "the TODOs above are unfilled, so there is nothing to publish. "
                "Work through notebooks/lane_detection_activity.ipynb first.")

        self._publish_segments(msg.header, detections)
        self._publish_debug(msg.header, cropped, edges, masks, detections)

    def _publish_segments(self, header, detections):
        """Normalise every segment and publish them as one SegmentList.

        Called every frame, even with no detections - an empty list is still
        a message ground projection should receive.
        """
        # TODO (Part III): build one SegmentList and publish it on self.pub_segments.
        #
        # * Copy only header.stamp onto the list, not the whole header.
        # * detections maps colour name -> (lines, normals): lines is Nx4
        #   (x1, y1, x2, y2) in CROPPED-image pixels, normals is Nx2.
        # * One Segment per line. `rosmsg show duckietown_msgs/Segment` lists
        #   its fields; SEGMENT_COLOR_IDS gives the colour id for each name.
        # * Segment coordinates are fractions of the image, not pixels. Ground
        #   projection applies the Lab 1 camera calibration, which describes
        #   the WHOLE frame, so undo the crop (self.cutoff_rows) before dividing
        #   by the resized size (self.img_w, self.img_h). Which height you
        #   divide by matters, and getting it wrong does not raise an error.
        pass

    # ----------------------------------------------------------------------
    # Provided from here down. Read it, but you should not need to edit it.
    # ----------------------------------------------------------------------
    def _switch(self, req):
        """Answer the FSM's switch service. We are always on."""
        return SetBoolResponse(True, "")

    def _orient(self, lines, mask):
        """Give every segment a direction, so its endpoint order means something.

        HoughLinesP hands back two endpoints in no particular order. That is a
        problem, because a lane marking has TWO edges - the road-side one and
        the far-side one - and they look identical to the Hough transform. What
        tells them apart is which side of the segment the paint is on.

        This matters downstream. `lane_filter` decides whether it is looking at
        the left or the right edge of the white line by comparing the two
        endpoints.

        Reorders `lines` in place and returns an Nx2 array of unit normals.
        """
        if len(lines) == 0:
            return np.zeros((0, 2))

        length = np.sum((lines[:, 0:2] - lines[:, 2:4]) ** 2,
                        axis=1, keepdims=True) ** 0.5
        length[length == 0] = 1.0

        # Unit perpendicular to each segment, and its midpoint.
        dx = 1.0 * (lines[:, 3:4] - lines[:, 1:2]) / length
        dy = 1.0 * (lines[:, 0:1] - lines[:, 2:3]) / length
        cx = (lines[:, 0:1] + lines[:, 2:3]) / 2.0
        cy = (lines[:, 1:2] + lines[:, 3:4]) / 2.0

        # Probe the mask either side of the midpoint.
        probe = self.normal_probe_px
        x3 = self._clamp(cx - probe * dx, mask.shape[1])
        y3 = self._clamp(cy - probe * dy, mask.shape[0])
        x4 = self._clamp(cx + probe * dx, mask.shape[1])
        y4 = self._clamp(cy + probe * dy, mask.shape[0])

        # +1 where the first probe is on paint and the second is not, else -1.
        signs = (np.logical_and(mask[y3, x3] > 0,
                                mask[y4, x4] == 0)).astype(int) * 2 - 1
        normals = np.hstack([dx, dy]) * signs

        # Swap the endpoints wherever direction and normal are the wrong way
        # round, so endpoint order alone records which side the paint is on.
        flip = ((lines[:, 2] - lines[:, 0]) * normals[:, 1]
                - (lines[:, 3] - lines[:, 1]) * normals[:, 0]) > 0
        lines[flip] = lines[flip][:, [2, 3, 0, 1]]

        return normals

    @staticmethod
    def _clamp(values, bound):
        """Round to int pixel indices and keep them inside [0, bound)."""
        out = values.astype(int)
        np.clip(out, 0, bound - 1, out=out)
        return out

    @staticmethod
    def _has_subscribers(publisher):
        return publisher.get_num_connections() > 0

    def _publish_debug(self, header, cropped, edges, masks, detections):
        if cropped is None:
            return

        if self._has_subscribers(self.pub_cropped):
            self._publish_image(self.pub_cropped, header, cropped, "bgr8")

        if edges is not None and self._has_subscribers(self.pub_edges):
            self._publish_image(self.pub_edges, header, edges, "mono8")

        for name, mask in masks.items():
            if self._has_subscribers(self.pub_masks[name]):
                self._publish_image(self.pub_masks[name], header, mask, "mono8")

        for name, (lines, normals) in detections.items():
            if self._has_subscribers(self.pub_lines[name]):
                drawn = self._draw_lines(cropped, lines, normals, DRAW_COLORS[name])
                self._publish_image(self.pub_lines[name], header, drawn, "bgr8")

        if self._has_subscribers(self.pub_lines_all):
            drawn = np.copy(cropped)
            for name, (lines, normals) in detections.items():
                drawn = self._draw_lines(drawn, lines, normals, DRAW_COLORS[name])
            self._publish_image(self.pub_lines_all, header, drawn, "bgr8")

    def _publish_image(self, publisher, header, image, encoding):
        msg = self.bridge.cv2_to_imgmsg(image, encoding)
        msg.header = header
        publisher.publish(msg)

    def _draw_lines(self, image, lines, normals, color):
        """Draw each segment, plus a stub showing which way its normal points.

        The stub is the useful part: it should always point away from the paint,
        on the same side for every segment of a given marking. If they disagree
        with each other, your orientation step is not working. The magenta dot
        is the segment's FIRST endpoint - which one it is is the whole subject
        of Part IV.
        """
        out = np.copy(image)
        for (x1, y1, x2, y2), normal in zip(lines, normals):
            cv2.line(out, (x1, y1), (x2, y2), color, 1, cv2.LINE_AA)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            tip = (int(cx + 2 * self.normal_probe_px * normal[0]),
                   int(cy + 2 * self.normal_probe_px * normal[1]))
            cv2.line(out, (cx, cy), tip, (0, 255, 0), 1)
            cv2.circle(out, (x1, y1), 1, (255, 0, 255), -1)
        return out


if __name__ == "__main__":
    try:
        LaneDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# The detector fed by saved images, with a viewer per debug topic.
# Run it from outside the container with:
#
#     dts devel run -X -L detector_test
#
# -X is what lets the rqt_image_view windows out of the container.
dt-exec roslaunch lane_detection lane_detection.launch test:=true

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join

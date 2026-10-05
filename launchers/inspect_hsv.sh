#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# The live HSV picker. Hover the cursor over the image to read pixel values.
#
#     dts devel run -X -R <robot> -L inspect_hsv    # your robot's camera
#     dts devel run -X -L inspect_hsv test:=true    # the sample images
#
# -X is required either way; without it no window appears.
dt-exec roslaunch camera_sim hsv_inspector.launch ${@}

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join

#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# The detector wired into the full Duckietown stack. THIS ONE RUNS ON THE BOT:
#
#     dts devel build -H <robot> -f
#     dts devel run   -H <robot> -L lane_following
#
# No -X here - the robot has no display. Open the debug views from `dts gui`.
dt-exec roslaunch lane_detection lane_following.launch

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join

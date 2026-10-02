# ECEN 433 - Lab 4: Lane Detection & Image Processing

Starter code for Lab 4. You will write one ROS node that finds the white,
yellow and red lane markings in a camera image and publishes them as line
segments. The same node runs against saved images on this machine and against
the live camera on your Duckiebot, where it replaces Duckietown's own line
detector and feeds their ground projection, lane filter and controller.


## Building and running

```bash
dts devel build -f
```

The workspace is installed with symlinks, so **editing** an existing Python node
or launch file does not need a rebuild - save the file and launch again. Re-run
the build when you **add** a new file, rename one, or add a package, message
type, or dependency.

Then, on this machine, against the sample images:

```bash
dts devel run -X -L detector_test
```

- `-X` allows the container to open GUI windows. Without it none of the
  `rqt_image_view` windows appear.
- `-L detector_test` runs `launchers/detector_test.sh`.

On your robot, wired into the Duckietown stack:

```bash
dts devel build -H DUCKIEBOT_NAME -f
dts devel run   -H DUCKIEBOT_NAME -L lane_following
```

No `-X` there - the robot has no screen. Open the debug views from a separate
`dts gui DUCKIEBOT_NAME` shell with `rqt_image_view`.

To poke around inside the container instead of launching straight away:

```bash
dts devel run -X --cmd bash
```

and to attach a second terminal to a container that is already running:

```bash
dts devel run attach
```

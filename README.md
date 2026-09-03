# table-hockey
A table hockey robot using parallel link mechanism and vision sensor

![picture of the robot](./overview_1.jpg)
![screenshot](./screenshot_1.png)

See https://yamaguchi-am.blogspot.com/2021/12/blog-post.html for details.

## Hardware Setup
- KONDO DUAL USB adapter
  - 2 KRS servos (ID: 0 and 1, baud: 1.25M)
- USB camera

## Software dependencies
- libgflags-dev
- libgtest-dev
- libeigen3-dev
- libopencv-dev
- setserial

## System setup

Set USB-serial device to low_latency mode.

```
setserial /dev/ttyUSB0 low_latency
```

Optionally, set it by udev rule. For example, for KONDO USB-DUAL adapter:
```
----[/etc/udev/rules.d/99-kondo.rules]----
ACTION=="add", ATTRS{idVendor}=="165c", ATTRS{idProduct}=="0008", GROUP="dialout", RUN+="/bin/sh -c 'echo 165C 0008 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id'", RUN+="/bin/sh -c 'setserial /dev/%k low_latency'", SYMLINK+="ttyUSB-KONDO_DUAL"
```

Note that `setserial` command must be installed before applying above rule.

## Build

    git submodule update --init
    mkdir build
    cd bulid
    cmake ..
    make

## Camera and Mechanism Calibration
1. Use [yamaguchi-am/opencv_calibration](https://github.com/yamaguchi-am/opencv_calibration) to generate camera parameter XML file from images.
  - The unit should be given in millimeters.
  - The coordinate system (decided by the 1st image) should be aligned to the field. See the blog for details.

2. Start this program, place a checker board pattern, and hit [F] to capture camera position.
  - This step may be omitted if the coordinate system is already aligned by step 1.

3. Collect hand-eye calibration datapoints by [a] key, (Optionally, save it by [S])

4. Enter "O (Optimize)" command and see if the blue line fits to the mechanism in the image. Note that each link line may no always exactly align with the real links of the mechanism, even when the end effector position is aligned.

5. Configure field size.
  - Place the ball to the left edge of the field and enter [1] command.
  - Place the ball to the right edge of the field and enter [2] command.
  - Attach the optical marker (ball) to the racket again, move it to the waiting position,
    then enter [3] command (set min_x).

6. Save the configuration by [W] command.

When the camera is relocated, go back to step 2.

When the camera is replaced, go back to step 1.

## Key bindings

- a: add hand-eye calibration data point.
- f: capture camera position; fit the world coordinate system to a chess board pattern.
- s: save hand-eye calibration data points to file (calib_points.txt). Deprecated. Use [W] to save arm parameters after optimization instead.
- l: load hand-eye calibration data points from file. Deprecated.
- w: save configuration (camera position, arm parameters, field borders) to file.
- r: load configuration from file.
- o: optimize hand-eye parameters using the data points.
- g: toggle ON/OFF arm motion
- 1: set field left (Y+) limit to the current ball position
- 2: set field right (Y-) limit to the current ball position
- 3: set field near (X-) limit to the current ball position

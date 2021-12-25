# table-hockey
A table hockey robot using parallel link mechanism and vision sensor

See https://yamaguchi-am.blogspot.com/2021/12/blog-post.html for details.

## How to use
TBD

- Use [yamaguchi-am/opencv_calibration](https://github.com/yamaguchi-am/opencv_calibration) to generate camera parameter XML file from images.
  - The unit should be given in millimeters.
  - The coordinate system (decided by the 1st image) should be aligned to the field. See the blog for details.

- hand-eye calibration data is not saved. Load calibration data points and optimize after starting the program.

## Key bindings

- a: add hand-eye calibration data point.
- S: save hand-eye calibration data points to file (calib_points.txt).
- L: load hand-eye calibration data points from file.
- O: optimize hand-eye parameters using the data points.
- g: toggle ON/OFF arm motion
- 1: set field left (Y+) limit to the current ball position
- 2: set field right (Y-) limit to the current ball position

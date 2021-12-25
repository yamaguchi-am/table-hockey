#ifndef CV_UTIL_H_
#define CV_UTIL_H_

#include <opencv2/opencv.hpp>

// Get pointer to the pixel data at (x, y).
unsigned char* PixelAt(cv::Mat m, int x, int y);

#endif  // CV_UTIL_H_

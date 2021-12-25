#ifndef BALL_DETECTOR_H_
#define BALL_DETECTOR_H_

#include <opencv2/opencv.hpp>

// Returns true when found.
bool DetectBall(cv::Mat& img, cv::Point2d* result);

#endif  // BALL_DETECTOR_H_

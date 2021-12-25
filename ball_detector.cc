#include "ball_detector.h"

#include <opencv2/opencv.hpp>
#include <vector>

#include "cv_util.h"

bool DetectBall(cv::Mat& img, cv::Point2d* result) {
  constexpr int kMinRegionSizePixels = 30;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  typedef std::pair<double, int> roundness_idx_pair;
  std::vector<roundness_idx_pair> candidates;
  for (size_t i = 0; i < contours.size(); i++) {
    const auto& c = contours.at(i);
    // There should be at least 5 points to fit the ellipse by fitEllipse
    if (c.size() < 5) {
      continue;
    }
    double area = cv::contourArea(c);
    if (area < kMinRegionSizePixels) {
      continue;
    }
    double l = cv::arcLength(c, true);
    // If the shape is circular:
    // area = (l/pi/2)**2 * pi = l**2 / pi / 4
    double x = l * l / M_PI / 4 / area;
    candidates.push_back(std::make_pair(fabs(x - 1), i));
  }
  auto best = std::min_element(
      candidates.begin(), candidates.end(),
      [](const roundness_idx_pair& a, const roundness_idx_pair& b) {
        return a.first < b.first;
      });
  if (best != candidates.end()) {
    cv::RotatedRect r = cv::fitEllipse(contours[best->second]);
    *result = r.center;
    return true;
  }
  return false;
}

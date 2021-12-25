#include "color_detector.h"

#include <ctype.h>

#include <vector>

#include "cv_util.h"

using namespace std;

namespace {

int AbsHueDiff(int h1, int h2) {
  int d = h1 - h2;
  if (d < 0) {
    d = -d;
  }
  if (d > 90) {
    d = 180 - d;
  }
  return d;
}

}  // namespace

ColorDetectorParam ColorDetectorParam::White(int minBrightness,
                                             int maxSaturation) {
  // sMin=0, hueMean=any, hueWidth>180
  ColorDetectorParam result = ColorDetectorParam(0, 0, 180, minBrightness);
  result.is_chromatic_ = false;
  result.saturation_ = maxSaturation;
  return result;
}

ColorDetectorParam ColorDetectorParam::Chroma(int sMin, int hueMean,
                                              int hueWidth, int minBrightness) {
  ColorDetectorParam result = ColorDetectorParam(sMin, hueMean, hueWidth);
  result.is_chromatic_ = true;
  result.minBrightness = minBrightness;  // not used
  return result;
}

cv::Mat ColorDetector::DetectAndReturnBinary(
    const cv::Mat& input, const ColorDetectorParam& param) const {
  cv::Mat binary(cv::Size(input.cols, input.rows), CV_8UC1);
  Detect(input, param, &binary);
  return binary;
}

void ColorDetector::Detect(const cv::Mat& frame,
                           const ColorDetectorParam& param,
                           cv::Mat* resultImage) const {
  cv::Mat hsv_image;
  cv::cvtColor(frame, hsv_image, cv::ColorConversionCodes::COLOR_BGR2HSV);
  for (int y = 0; y < hsv_image.rows; y++) {
    for (int x = 0; x < hsv_image.cols; x++) {
      unsigned char* p = PixelAt(hsv_image, x, y);
      if (param.Match(p)) {
        PixelAt(*resultImage, x, y)[0] = 255;
      } else {
        PixelAt(*resultImage, x, y)[0] = 0;
      }
    }
  }
}

bool ColorDetectorParam::Match(unsigned char* hsv) const {
  int h = hsv[0];
  int s = hsv[1];
  int v = hsv[2];
  if (is_chromatic_) {
    return (v >= minBrightness && s > saturation_ &&
            AbsHueDiff(h, hueMean) <= hueWidth);
  } else {
    return v >= minBrightness && s < saturation_;
  }
}

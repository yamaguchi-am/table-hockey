#ifndef COLOR_DETECTOR_H_
#define COLOR_DETECTOR_H_

#include <string.h>

#include <opencv2/opencv.hpp>
#include <sstream>

struct ColorDetectorParam {
  bool is_chromatic_;  // if true, s >= saturation. otherwise s<=saturation
  int saturation_;
  int hueMean;
  int hueWidth;
  int minBrightness;

 public:
  ColorDetectorParam(int sMin, int hueMean, int hueWidth, int minBrightness = 0)
      : is_chromatic_(true),
        saturation_(sMin),
        hueMean(hueMean),
        hueWidth(hueWidth),
        minBrightness(minBrightness) {}
  static ColorDetectorParam White(int minBrightness, int sMax);
  static ColorDetectorParam Chroma(int sMin, int hueMean, int hueWidth,
                                   int minBrightness = 30);
  std::string ToString() {
    std::ostringstream ostream;
    if (is_chromatic_) {
      ostream << "sMin,hMean,hWidth = " << saturation_ << "," << hueMean << ","
              << hueWidth;
    } else {
      ostream << "maxS,minV = " << saturation_ << "," << minBrightness;
    }
    return ostream.str();
  }
  bool Match(unsigned char* hsv) const;
};

class ColorDetector {
 public:
  void Detect(const cv::Mat& frame, const ColorDetectorParam& param,
              cv::Mat* resultImage) const;
  cv::Mat DetectAndReturnBinary(const cv::Mat& frame,
                                const ColorDetectorParam& param) const;
};

#endif  // COLOR_DETECTOR_H_

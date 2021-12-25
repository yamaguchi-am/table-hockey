#include "cv_util.h"

#include <opencv2/opencv.hpp>

unsigned char* PixelAt(cv::Mat m, int x, int y) {
  return &m.data[y * m.step + x * m.elemSize()];
}

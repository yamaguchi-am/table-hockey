#include "motion.h"

#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

TEST(MotionManagerTest, Idle) {
  MotionManager m(cv::Point2d(100, 100), 1000.0);
  m.SetDest(cv::Point2d(200, 100), 100);
  cv::Point2d p = m.Get(1000.1);
  ASSERT_FLOAT_EQ(110, p.x);
  p = m.Get(1001.0);
  ASSERT_FLOAT_EQ(200, p.x);
  p = m.Get(1011.0);
  ASSERT_FLOAT_EQ(200, p.x);
}

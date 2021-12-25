#include "motion.h"

#include <mutex>
#include <opencv2/opencv.hpp>

MotionManager::MotionManager(const cv::Point2d& init, double time) {
  current_pos_ = init;
  last_time_ = time;  // gettimeofdayInSeconds();
}

void MotionManager::SetPos(const cv::Point2d& p, double time) {
  std::lock_guard<std::mutex> lock(mtx_);
  current_pos_ = p;
  target_pos_ = p;
  last_time_ = time;
}

void MotionManager::SetDest(const cv::Point2d& p, double speed) {
  std::lock_guard<std::mutex> lock(mtx_);
  target_pos_ = p;
  speed_ = speed;
}

cv::Point2d MotionManager::Get(double current_time) {
  std::lock_guard<std::mutex> lock(mtx_);
  double passed = current_time - last_time_;
  last_time_ = current_time;
  auto v = target_pos_ - current_pos_;
  double dt = cv::norm(v) / speed_;
  if (dt > passed) {
    current_pos_ += passed * speed_ * (v / cv::norm(v));
  } else {
    current_pos_ = target_pos_;
  }
  return current_pos_;
}

#ifndef MOTION_H_
#define MOTION_H_

#include <mutex>
#include <opencv2/opencv.hpp>

struct Waypoint {
  cv::Point2d pos;
  double speed;
};

class MotionManager {
 public:
  MotionManager(const cv::Point2d& init, double current_time);
  void SetPos(const cv::Point2d& pos, double current_time);
  void SetDest(const cv::Point2d& p, double speed);
  cv::Point2d Get(double current_time);

 private:
  std::mutex mtx_;
  cv::Point2d current_pos_;
  cv::Point2d target_pos_;
  double speed_;
  double last_time_;
};

#endif  // MOTION_H_

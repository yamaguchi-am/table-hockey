#ifndef HOCKEY_H_
#define HOCKEY_H__

#include <Eigen/Dense>

#include "motion_estimator.h"

namespace {

Eigen::Vector3d PointForEstimator(cv::Point3d pt) {
  Eigen::Vector3d result;
  result[0] = pt.x;
  result[1] = pt.y;
  result[2] = pt.z;
  return result;
}

cv::Point3d PointFromEstimator(const Eigen::Vector3d& v) {
  return cv::Point3d{v[0], v[1], v[2]};
}

}  // namespace

struct FieldConfig {
  double y_max;
  double y_min;
};

class HockeyEstimator {
 public:
  HockeyEstimator(double history_length, int history_num,
                  const FieldConfig& field)
      : est_(history_length, history_num),
        ymax_(field.y_max),
        ymin_(field.y_min) {}

  void Accumulate(const cv::Point3d& pt, double current_time) {
    est_.Accumulate(PointForEstimator(pt), current_time);
  }

  bool Estimate(double time, cv::Point3d* result) {
    auto est = est_.Estimate(time);
    if (est.confidence == 0) {
      return false;
    }
    *result = PointFromEstimator(est.pt);
    if (result->y < ymin_) {
      result->y = ymin_ + (ymin_ - result->y);
    } else if (result->y > ymax_) {
      result->y = ymax_ + (ymax_ - result->y);
    }
    return true;
  }

  void UpdateFieldConfig(const FieldConfig& field) {
    ymax_ = field.y_max;
    ymin_ = field.y_min;
  }

 private:
  Estimator<Eigen::Vector3d> est_;
  double ymax_;
  double ymin_;
};

#endif  // HOCKEY_H_

#include "player.h"

#include <vector>

#include "arm.h"
#include "hockey.h"
#include "motion.h"

std::vector<double> Player::DecideMove(bool found, const cv::Point3d& ball,
                                       Waypoint* result) {
  cv::Point2d pos;
  const double kHomeX = -80;
  const double kHomeY = (field_.y_max + field_.y_min) / 2;
  const double kBallDiameter = 40;
  const double kRacketDiameter = 70;
  const double kHittingOverlap = 10;
  const double kXNear = 70;
  double margin = (kRacketDiameter - kBallDiameter) / 2 * 1.2;
  if (!found) {
    pos.x = kHomeX;
    pos.y = kHomeY;
    result->speed = idle_speed_;
  } else if (ball.x < kXNear) {
    pos.x = ball.x - kBallDiameter / 2 - kRacketDiameter / 2 + kHittingOverlap;
    pos.y = ball.y;
    result->speed = hit_speed_;
  } else {
    pos.x = kHomeX;
    pos.y = ball.y;
    result->speed = chase_speed_;
  }
  double y_min = field_.y_min + margin;
  double y_max = field_.y_max - margin;
  if (pos.y < y_min) {
    pos.y = y_min;
  } else if (pos.y > y_max) {
    pos.y = y_max;
  }
  result->pos = pos;
  if (!IsReachable(arm_, pos)) {
    pos.x = kHomeX;
    pos.y = kHomeY;
    result->speed = idle_speed_;
    result->pos = pos;
  }
  std::vector<double> angles;
  bool valid = Inverse(arm_, pos, &angles);
  assert(valid);
  return angles;
}

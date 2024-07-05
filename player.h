#ifndef PLAYER_H_
#define PLAYER_H_

#include "arm.h"
#include "hockey.h"
#include "motion.h"

class Player {
 public:
  Player(int idle_speed, int chase_speed, int hit_speed, const ArmConfig& arm,
         const FieldConfig& field)
      : idle_speed_(idle_speed),
        chase_speed_(chase_speed),
        hit_speed_(hit_speed),
        arm_(arm),
        field_(field) {}
  std::vector<double> DecideMove(bool found, const cv::Point3d& ball,
                                 Waypoint* result);

 private:
  int idle_speed_;
  int chase_speed_;
  int hit_speed_;
  const ArmConfig& arm_;
  const FieldConfig& field_;
};

#endif  // PLAYER_H_

#include "kondo_servo_driver.h"

#include <math.h>

#include <vector>

#include "kondo-ics/kondo_ics.h"

namespace {

constexpr int kServoPosCenter = 7500;
constexpr int kServoPosPerDeg = 30;

}  // namespace

void KondoServoDriver::Initialize() {
  online_ = true;
  for (int i = 0; i < 2; i++) {
    if (ics_.SetStretch(0, 30) != KondoIcs::SUCCESS) {
      online_ = false;
    };
  }
}

void KondoServoDriver::SendCommand(std::vector<double> angles,
                                   std::vector<double>* current_angles) {
  for (int i = 0; i < 2; i++) {
    uint16_t current;
    int v = kServoPosCenter + angles.at(i) * kServoPosPerDeg * -180 / M_PI;
    ics_.SetPos(i, v, &current);
    double angle = (v - kServoPosCenter) / kServoPosPerDeg / -180.0 * M_PI;
    (*current_angles)[i] = angle;
  }
}

void KondoServoDriver::FreeAndCapture(
    std::vector<double>* current_angles) const {
  for (int i = 0; i < 2; i++) {
    uint16_t v;
    if (ics_.SetPos(i, 0, &v) == KondoIcs::SUCCESS) {
      double angle = (v - kServoPosCenter) / kServoPosPerDeg / -180.0 * M_PI;
      (*current_angles)[i] = angle;
    }
  }
}

void KondoServoDriver::FreeAll() {
  uint16_t unused;
  ics_.SetPos(0, 0, &unused);
  ics_.SetPos(1, 0, &unused);
}

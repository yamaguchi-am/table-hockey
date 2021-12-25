#ifndef KONDO_SERVO_DRIVER_
#define KONDO_SERVO_DRIVER_

#include <vector>

#include "kondo-ics/kondo_ics.h"

class KondoServoDriver {
 public:
  KondoServoDriver(const std::string& port, int baudrate)
      : ics_(KondoIcs(port, baudrate)) {
    Initialize();
  }

  void Initialize();
  void SendCommand(std::vector<double> angles,
                   std::vector<double>* current_angles);

  // Turn off and receive current position.
  // (Old firmware of KONDO KRS servo does not have position capture command)
  void FreeAndCapture(std::vector<double>* current_angles) const;
  void FreeAll();
  bool IsOnline() const { return online_; }

 private:
  KondoIcs ics_;
  bool online_;
};

#endif  // KONDO_SERVO_DRIVER_

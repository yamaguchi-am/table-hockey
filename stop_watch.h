#ifndef STOP_WATCH_H_
#define STOP_WATCH_H_

#include <stdlib.h>
#include <sys/time.h>

class StopWatch {
 private:
  double startTime;
  double lap_start_time_;
  static double gettimeofday_sec() {
    struct timeval t;
    gettimeofday(&t, NULL);
    return (double)t.tv_sec + (double)t.tv_usec * 1e-6;
  }

 public:
  StopWatch() { start(); }
  void start() {
    startTime = gettimeofday_sec();
    lap_start_time_ = startTime;
  }
  double Peek() {
    double current = gettimeofday_sec();
    double lap_time = current - lap_start_time_;
    lap_start_time_ = current;
    return lap_time;
  }
  double stop() {
    double current = gettimeofday_sec();
    double prev = startTime;
    startTime = current;
    return current - prev;
  }
};

#endif  // STOP_WATCH_H_

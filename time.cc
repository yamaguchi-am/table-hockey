#include "time.h"

#include <sys/time.h>

#include <cstddef>

double GettimeofdayInSeconds() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return (double)t.tv_sec + (double)t.tv_usec * 1e-6;
}

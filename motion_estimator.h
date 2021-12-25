#ifndef MOTION_ESTIMATOR_H_
#define MOTION_ESTIMATOR_H_

#include <Eigen/Dense>
#include <deque>
#include <iostream>
#include <vector>

// T should support +, * operators and vector components accessible by
// operator()
template <class T>
struct EstimatorResult {
  double confidence;
  T pt;
};

template <typename T>
class HistoryRow {
 public:
  double t;
  T data;
  HistoryRow(double t0, T data0) : t(t0), data(data0) {}
};

class History {
 public:
  virtual double GetDataAt(int i) const = 0;
  virtual double GetTimeAt(int i) const = 0;
  virtual size_t size() const = 0;
};

template <typename T>
class PosHistory : public History {
 public:
  PosHistory(const std::deque<HistoryRow<T>>& data, int idx)
      : data_(data), idx_(idx) {}
  double GetDataAt(int i) const { return data_.at(i).data(idx_); }
  double GetTimeAt(int i) const { return data_.at(i).t; }
  size_t size() const { return data_.size(); }

 private:
  const std::deque<HistoryRow<T>>& data_;
  size_t idx_;
};

double Exterpolate(const History& data, double time, double* result_error) {
  const int kOrder = 1;  // Fit a 2nd order function
  const int n = data.size();
  assert(n > 0);
  // Number of coefficients is order+1. Need as many data points as that.
  if (n < kOrder + 2) {
    return data.GetDataAt(n - 1);
  }
  Eigen::Matrix<double, Eigen::Dynamic, kOrder + 1> mat_A(n, kOrder + 1);
  Eigen::Matrix<double, Eigen::Dynamic, 1> vec_b(n, 1);
  const double& t0 = data.GetTimeAt(0);
  for (int i = 0; i < n; i++) {
    const double t = data.GetTimeAt(i) - t0;
    double p = 1;
    for (int o = 0; o <= kOrder; o++) {
      mat_A(i, o) = p;
      p *= t;
    }
    vec_b[i] = data.GetDataAt(i);
  }
  auto coeff =
      (mat_A.transpose() * mat_A).inverse() * mat_A.transpose() * vec_b;
  double p = 1;
  double result = 0;
  time -= t0;
  for (int i = 0; i < kOrder + 1; i++) {
    result += p * coeff[i];
    p *= time;
  }
  return result;
}

// T should be vector type
template <class T>
class Estimator {
 public:
  Estimator(double max_duration, int max_element_count)
      : max_duration_(max_duration), max_element_count_(max_element_count) {}
  void Accumulate(const T& data, double time) {
    history_.emplace_back(HistoryRow<T>(time, data));
    RemoveStalePoints(time);
    while (history_.size() >= max_element_count_) {
      history_.pop_front();
    }
  }

  void RemoveStalePoints(double time) {
    while (!history_.empty() && history_.front().t < time - max_duration_) {
      history_.pop_front();
    }
  }

  EstimatorResult<T> Estimate(double time) const {
    EstimatorResult<T> result;
    if (history_.size() < 2) {
      result.confidence = 0;
      return result;
    }
    for (int i = 0; i < 3; i++) {
      PosHistory<T> d(history_, i);
      double err;
      double est = Exterpolate(d, time, &err);
      result.pt[i] = est;
    }
    result.confidence = 1.0;
    return result;
  }

 private:
  std::deque<HistoryRow<T>> history_;
  double max_duration_;
  size_t max_element_count_;
};

#endif  // MOTION_ESTIMATOR_H_

#ifndef ARM_H_
#define ARM_H_

#include <opencv2/core.hpp>
#include <vector>

/*
 Config represents geometric parameters of the robot.

          s
      m1  |  m2
   a------r------b
l1 |             | l2
   p             q

 b, r, and s are on the same link.
 a1 = angle(br,bs)
 n = distance(b,s)

 */
struct ArmConfig {
 public:
  static ArmConfig Default();

  void Encode(double* x) const;
  void Decode(const double* x);

  void WriteToFileStorage(cv::FileStorage& fs) const;
  void ReadFromFileNode(cv::FileNode node);

  cv::Point2d p;  // right base
  cv::Point2d q;  // left base
  double l1;      // right upper arm
  double l2;      // left upper arm
  double m1;      // right lower arm
  double m2;      // left lower arm
  double n;       // length from right elbow to end effector
  double a1;  // end effector offset angle relative to the link (see the drawing
              // above)
  double trim0;
  double trim1;
};

struct ArmVertices {
  cv::Point2d p;
  cv::Point2d q;
  cv::Point2d a;
  cv::Point2d b;
  cv::Point2d r;
  cv::Point2d s;
};

struct CalibrationDataPoint {
  double x;
  double y;
  double angle[2];
};

class CalibrationData {
 public:
  std::vector<CalibrationDataPoint> data;
  void Save(const std::string& filename) const;
  void Load(const std::string& filename);
  void Dump() const;
};

ArmVertices Forward(const ArmConfig& config, std::vector<double> angles);
bool Inverse(const ArmConfig& config, const cv::Point2d& s,
             std::vector<double>* result);
bool IsReachable(const ArmConfig& config, cv::Point2d pos);

int Optimize(CalibrationData& pts, const ArmConfig& init, ArmConfig* output);

#endif  // ARM_H_

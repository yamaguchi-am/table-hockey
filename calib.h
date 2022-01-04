#ifndef CV_CALIB_H_
#define CV_CALIB_H_

#include <opencv2/core/core.hpp>
#include <string>

class Calib {
 private:
  cv::Mat t;
  cv::Mat r;
  // The offset added to triangulation result.
  double yOffset;

 public:
  cv::Mat intrinsic;
  cv::Mat distortion;
  Calib() {}
  Calib(const Calib& src) {
    t = src.t.clone();
    r = src.r.clone();
    yOffset = src.yOffset;
    intrinsic = src.intrinsic.clone();
    distortion = src.distortion.clone();
  }
  Calib(const char* filename, double yOffset = 0);
  Calib(const std::string& filename, double yOffset = 0);

  void SetExtrinsic(const cv::Mat& r, const cv::Mat& t) {
    this->t = t;
    this->r = r;
  }

  void Load(const char* filename, double yOffset = 0);
  void WriteToFileStorage(cv::FileStorage& fs) const;
  void ReadFromFileNode(cv::FileNode fs);

  bool GetPlaneCrossPoint(double x, double y, double floorHeight,
                          cv::Point3d* result) const;
  cv::Point2d Project(cv::Point3d point3d) const;
  cv::Mat Undistort(const cv::Mat& image) const;
};

#endif  // CV_CALIB_H_

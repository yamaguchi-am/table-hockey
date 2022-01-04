#include "calib.h"

#include <stddef.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

Calib::Calib(const std::string& filename, double yOffset) {
  Load(filename.c_str(), yOffset);
}

Calib::Calib(const char* filename, double yOffset) { Load(filename, yOffset); }

void Calib::Load(const char* filename, double yOffset) {
  this->yOffset = yOffset;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "failed to open file: " << filename << std::endl;
    exit(-1);
  }
  ReadFromFileNode(fs.root());
}

void Calib::WriteToFileStorage(cv::FileStorage& fs) const {
  fs << "intrinsic" << intrinsic;
  fs << "distortion" << distortion;
  cv::Mat rvec;
  cv::Rodrigues(r, rvec);
  fs << "rotation" << rvec;
  fs << "translation" << t;
}

void Calib::ReadFromFileNode(cv::FileNode fs) {
  fs["intrinsic"] >> intrinsic;
  fs["distortion"] >> distortion;
  cv::Mat rvec;
  fs["rotation"] >> rvec;
  cv::Rodrigues(rvec, r);
  fs["translation"] >> t;
}

// x, y: undistorted image point
bool Calib::GetPlaneCrossPoint(double x, double y, double floorHeight,
                               cv::Point3d* result) const {
  cv::Mat cam_center = -r.t() * t;

  // find t | camCenter + t * viewVector is on the floor plane
  cv::Mat viewVector(3, 1, CV_64FC1);
  cv::Mat a(1, 1, CV_32FC2);
  a.at<cv::Vec2f>(0, 0)[0] = x;
  a.at<cv::Vec2f>(0, 0)[1] = y;
  cv::undistortPoints(a, a, intrinsic, distortion);
  viewVector.at<double>(0, 0) = a.at<cv::Vec2f>(0, 0)[0];
  viewVector.at<double>(1, 0) = a.at<cv::Vec2f>(0, 0)[1];
  viewVector.at<double>(2, 0) = 1;
  viewVector = r.t() * viewVector;
  cv::Mat z = (cv::Mat_<double>(3, 1) << 0, 0, 1);
  float alpha = -(cam_center.dot(z) - floorHeight) / viewVector.dot(z);
  if (alpha < 0) {
    return false;
  }
  cv::Mat floorPoint = cam_center + alpha * viewVector;
  result->x = floorPoint.at<double>(0, 0);
  result->y = floorPoint.at<double>(1, 0) - yOffset;
  result->z = floorPoint.at<double>(2, 0);
  return true;
}

cv::Point2d Calib::Project(cv::Point3d point3d) const {
  vector<cv::Point2f> imagePoints;
  vector<cv::Point3f> object;
  object.push_back(cv::Point3f(point3d.x, point3d.y - yOffset, point3d.z));
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);
  cv::Rodrigues(r, rvec);
  cv::Mat tvec = t;
  cv::projectPoints(object, rvec, tvec, intrinsic, distortion, imagePoints);
  double x = imagePoints.at(0).x;
  double y = imagePoints.at(0).y;
  return cv::Point2d(x, y);
}

cv::Mat Calib::Undistort(const cv::Mat& src) const {
  cv::Mat dest;
  cv::undistort(src, dest, intrinsic, distortion);
  return dest;
}

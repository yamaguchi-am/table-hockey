#include <gflags/gflags.h>
#include <unistd.h>

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "arm.h"
#include "ball_detector.h"
#include "calib.h"
#include "color_detector.h"
#include "hockey.h"
#include "kondo_servo_driver.h"
#include "motion.h"
#include "stop_watch.h"
#include "time.h"

namespace {

const std::string kMainWindow = "preview";

}  // namespace

DEFINE_int32(cam_id, 0, "camera ID");
DEFINE_string(cam_calib, "../data/vga.xml", "camera calibration XML file");
DEFINE_string(port, "/dev/ttyUSB0", "serial port for KONDO ICS adapter");
DEFINE_bool(ignore_servo_offline, true,
            "continue even when servos are not responding at startup");

void DrawAxis(const Calib& calib, cv::Mat* img) {
  cv::Point2d o = calib.Project(cv::Point3d(0, 0, 0));
  cv::Point2d pt_x = calib.Project(cv::Point3d(50, 0, 0));
  cv::Point2d pt_y = calib.Project(cv::Point3d(0, 50, 0));
  cv::line(*img, o, pt_x, cv::Scalar(255, 0, 0), 2);
  cv::line(*img, o, pt_y, cv::Scalar(0, 0, 255), 2);
}

void DrawField(const Calib& calib, const FieldConfig& field, cv::Mat* img) {
  double kGridSize = 100;
  auto line = [&](double x0, double y0, double x1, double y1, cv::Scalar color,
                  double thickness) {
    constexpr double z = 0;
    cv::Point2d p0 = calib.Project(cv::Point3d(x0, y0, z));
    cv::Point2d p1 = calib.Project(cv::Point3d(x1, y1, z));
    cv::line(*img, p0, p1, color, thickness);
  };
  auto border_color = cv::Scalar(255, 255, 255);
  auto grid_color = cv::Scalar(0, 255, 0);
  double kXmin = -100;
  double kXmax = 500;  // TODO: replace this with camera view limit

  DrawAxis(calib, img);

  line(kXmin, field.y_min, kXmax, field.y_min, border_color, 2);
  line(kXmin, field.y_max, kXmax, field.y_max, border_color, 2);
  if (field.y_min > field.y_max) {
    // The configuration is invalid (e.g. left and right swapped)
    return;
  }
  for (double y = ceil(field.y_min / kGridSize) * kGridSize;
       y <= floor(field.y_max / kGridSize) * kGridSize; y += kGridSize) {
    line(kXmin, y, kXmax, y, grid_color, 1);
  }
  for (double x = ceil(kXmin / kGridSize) * kGridSize;
       x <= floor(kXmax / kGridSize) * kGridSize; x += kGridSize) {
    line(x, field.y_min, x, field.y_max, grid_color, 1);
  }
}

void DrawArm(const ArmConfig& config, const std::vector<double>& angles,
             const Calib& calib, cv::Scalar color, cv::Mat* img) {
  auto vertices = Forward(config, angles);

  double z = 20;
  cv::Point2d p = calib.Project(cv::Point3d(vertices.p.x, vertices.p.y, z));
  cv::Point2d q = calib.Project(cv::Point3d(vertices.q.x, vertices.q.y, z));
  cv::Point2d a = calib.Project(cv::Point3d(vertices.a.x, vertices.a.y, z));
  cv::Point2d b = calib.Project(cv::Point3d(vertices.b.x, vertices.b.y, z));
  cv::Point2d r = calib.Project(cv::Point3d(vertices.r.x, vertices.r.y, z));
  cv::Point2d s = calib.Project(cv::Point3d(vertices.s.x, vertices.s.y, z));
  double thickness = 3;
  cv::line(*img, p, a, color, thickness);
  cv::line(*img, q, b, color, thickness);
  cv::line(*img, a, r, color, thickness);
  cv::line(*img, b, r, color, thickness);
  cv::line(*img, r, s, color, thickness);
}

std::vector<double> DecideMove(bool found, const ArmConfig& config,
                               const cv::Point3d& pt3d, Waypoint* result,
                               const FieldConfig& field) {
  cv::Point2d pos;
  const double kHomeX = -80;
  const double kHomeY = 150;
  const double kNormalSpeed = 400;
  const double kMaxSpeed = 2000;
  const double kBallDiameter = 40;
  const double kRacketDiameter = 70;
  const double kHittingOverlap = 10;
  const double kXNear = 70;
  double margin = (kRacketDiameter - kBallDiameter) / 2 * 1.2;
  if (!found) {
    pos.x = kHomeX;
    pos.y = kHomeY;
    result->speed = kNormalSpeed;
  } else if (pt3d.x < kXNear) {
    pos.x = pt3d.x - kBallDiameter / 2 - kRacketDiameter / 2 + kHittingOverlap;
    pos.y = pt3d.y;
    result->speed = kMaxSpeed;
  } else {
    pos.x = kHomeX;
    pos.y = pt3d.y;
    result->speed = kMaxSpeed;
  }
  double y_min = field.y_min + margin;
  double y_max = field.y_max - margin;
  if (pos.y < y_min) {
    pos.y = y_min;
  } else if (pos.y > y_max) {
    pos.y = y_max;
  }
  result->pos = pos;
  if (!IsReachable(config, pos)) {
    pos.x = kHomeX;
    pos.y = kHomeY;
    result->speed = kNormalSpeed;
    result->pos = pos;
  }
  std::vector<double> angles;
  bool valid = Inverse(config, pos, &angles);
  assert(valid);
  return angles;
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const int kImageWidth = 640;
  const int kImageHeight = 480;

  Calib calib(FLAGS_cam_calib, 0);
  cv::Mat map1;
  cv::Mat map2;
  cv::initUndistortRectifyMap(
      calib.intrinsic, calib.distortion, cv::noArray(), calib.intrinsic,
      cv::Size(kImageWidth, kImageHeight), CV_32FC1, map1, map2);

  cv::VideoCapture capture(FLAGS_cam_id);
  if (!capture.isOpened()) {
    std::cerr << "failed to open video capture device" << std::endl;
    return -1;
  }
  capture.set(cv::CAP_PROP_FRAME_WIDTH, kImageWidth);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, kImageHeight);

  KondoServoDriver servos(FLAGS_port, 1250000);
  if (!servos.IsOnline() && !FLAGS_ignore_servo_offline) {
    std::cerr << "failed to communicate with servo" << std::endl;
    return -1;
  }
  MotionManager mm(cv::Point2d(0, 0), GettimeofdayInSeconds());

  // TODO: Save to / Load from data file.
  FieldConfig field;
  field.y_max = 348;
  field.y_min = -81;

  // TODO: Save to / Load from data file.
  ArmConfig config = ArmConfig::Default();

  bool active = false;
  bool quit = false;
  std::vector<double> current_angles(2);
  std::mutex servo_mtx;

  HockeyEstimator estimator(0.2, 60 * 0.2, field);
  std::thread servo_thread([&] {
    while (!quit) {
      double current_time = GettimeofdayInSeconds();
      std::vector<double> ik_angles;
      bool success = Inverse(config, mm.Get(current_time), &ik_angles);
      assert(success);
      servo_mtx.lock();
      if (active) {
        servos.SendCommand(ik_angles, &current_angles);
      } else {
        servos.FreeAndCapture(&current_angles);
      }
      servo_mtx.unlock();
      usleep(100);
    }
  });

  cv::Mat raw_image;
  cv::Mat undistort;
  ColorDetector detector;
  // TODO: Load parameters from configuration file.
  auto param = ColorDetectorParam::Chroma(150, 10, 5);
  CalibrationData calib_data;
  cv::namedWindow(kMainWindow);

  while (true) {
    StopWatch main_loop_watch;
    double current_time = GettimeofdayInSeconds();
    capture >> raw_image;
    StopWatch detector_watch;
    cv::remap(raw_image, undistort, map1, map2,
              cv::InterpolationFlags::INTER_LINEAR);
    cv::Point2d pt;
    cv::Mat small;
    cv::pyrDown(undistort, small);
    cv::Mat result = detector.DetectAndReturnBinary(small, param);
    cv::Point2d pt2d;
    bool found = DetectBall(result, &pt2d);
    double detect_time = detector_watch.stop();

    pt2d *= 2;
    Waypoint wp;
    DrawField(calib, field, &undistort);
    cv::Point3d pt3d;
    cv::Point3d est_pt;
    if (found) {
      calib.GetPlaneCrossPoint(pt2d.x, pt2d.y, 0.020, &pt3d);
      cv::circle(undistort, calib.Project(pt3d), 10, cv::Scalar(255, 255, 255),
                 2);
      estimator.Accumulate(pt3d, current_time);
      if (estimator.Estimate(current_time + 1.0 / 30 * 3, &est_pt)) {
        cv::circle(undistort, calib.Project(est_pt), 10,
                   cv::Scalar(0, 255, 255), 2);
        DecideMove(found, config, est_pt, &wp, field);
      } else {
        Waypoint wp;
        DecideMove(found, config, pt3d, &wp, field);
      }
    } else {
      // move to home position
      DecideMove(found, config, pt3d, &wp, field);
    }
    if (active) {
      mm.SetDest(wp.pos, wp.speed);
    }

    servo_mtx.lock();
    DrawArm(config, current_angles, calib, cv::Scalar(255, 128, 128),
            &undistort);
    servo_mtx.unlock();

    char buf[256];
    snprintf(buf, 256, "%6.3lf %6.3f", main_loop_watch.stop(), detect_time);
    cv::putText(undistort, buf, cv::Point(0, 10), cv::FONT_HERSHEY_PLAIN, 1,
                cv::Scalar(255, 255, 255), 2);
    cv::imshow(kMainWindow, undistort);
    int k = cv::waitKey(1);
    if (k == 0x1b) {
      break;
    }
    switch (k) {
      case 'a':
        if (found) {
          calib_data.data.push_back(CalibrationDataPoint{
              pt3d.x, pt3d.y, current_angles[0], current_angles[1]});
        }
        break;
      case 'S':
        calib_data.Save("calib_points.txt");
        break;
      case 'L':
        calib_data.Load("calib_points.txt");
        break;
      case 'O':
        Optimize(calib_data, config, &config);
        break;
      case 'g':
        active = !active;
        break;
      case '1':
        field.y_min = pt3d.y;
        estimator.UpdateFieldConfig(field);
        break;
      case '2':
        field.y_max = pt3d.y;
        estimator.UpdateFieldConfig(field);
        break;
    }
  }
  quit = true;
  servo_thread.join();
  servos.FreeAll();
  return 0;
}
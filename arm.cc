#include "arm.h"

#include <fstream>
#include <iostream>

#include "third_party/cminpack/cminpack.h"

namespace {

cv::Point2d unit(double dir) { return cv::Point2d(cos(dir), sin(dir)); }

double squ(double x) { return x * x; }

// Returns cos(C) of the triangle formed by line segments whose lengths are
// {a,b,c}
double Yogen(double a, double b, double c) {
  return (squ(a) + squ(b) - squ(c)) / (2 * a * b);
}

// Cost function for cminpack lmdif().
//__cminpack_type_fcn_mn__(void *p, int m, int n, const __cminpack_real__ *x,
//__cminpack_real__ *fvec, int iflag );
int fcn(void* p, int m, int n, const double* x, double* fvec, int iflag) {
  ArmConfig config;
  config.Decode(x);
  auto data = (const CalibrationData*)p;
  int i = 0;
  for (const auto& p : data->data) {
    std::vector<double> angles;
    angles.push_back(p.angle[0]);
    angles.push_back(p.angle[1]);
    ArmVertices v = Forward(config, angles);
    fvec[i] = v.s.x - p.x;
    fvec[i + 1] = v.s.y - p.y;
    i += 2;
  }
  return 0;
}

}  // namespace

ArmVertices Forward(const ArmConfig& config, std::vector<double> angles) {
  ArmVertices result;
  result.p = config.p;
  result.q = config.q;
  result.a = result.p + config.l1 * unit(angles[0] + config.trim0);
  result.b = result.q + config.l2 * unit(angles[1] + config.trim1);
  auto ab = result.b - result.a;
  double l = cv::norm(ab);
  double phi =
      acos((squ(config.m1) + squ(l) - squ(config.m2)) / (2 * config.m1 * l));
  double atobdir = atan2(ab.y, ab.x);
  result.r = result.a + config.m1 * unit(atobdir - phi);
  result.s = result.a + config.n * unit(atobdir - phi + config.a1);
  return result;
}

bool Inverse(const ArmConfig& config, const cv::Point2d& s,
             std::vector<double>* result) {
  result->resize(2);
  auto ps = s - config.p;
  double r0 = cv::norm(ps);
  if (r0 > config.l1 + config.n) {
    return false;
  }
  double aps = acos(Yogen(config.l1, r0, config.n));
  double dir_s = atan2(ps.y, ps.x);
  double dir_a = dir_s - aps;
  (*result)[0] = dir_a - config.trim0;
  auto a = config.p + config.l1 * unit(dir_a);
  auto as = s - a;
  double as_dir = atan2(as.y, as.x);
  double ar_dir = as_dir - config.a1;
  auto r = a + config.m1 * unit(ar_dir);
  auto qr = r - config.q;
  if (norm(qr) > config.l2 + config.m2) {
    return false;
  }
  double rqb = acos(Yogen(norm(qr), config.l2, config.m2));
  double dir_qr = atan2(qr.y, qr.x);
  double dir_qb = dir_qr + rqb;
  (*result)[1] = dir_qb - config.trim1;
  return true;
}

bool IsReachable(const ArmConfig& config, cv::Point2d pos) {
  std::vector<double> angles;
  bool success = Inverse(config, pos, &angles);
  if (!success) {
    return false;
  }
  for (int i = 0; i < 2; i++) {
    double angle = angles[i];
    if (angle < -M_PI / 3 * 2 || angle > M_PI / 3 * 2) {
      return false;
    }
  }
  return true;
}

ArmConfig ArmConfig::Default() {
  ArmConfig config;
  config.a1 = 0.01;
  config.l1 = 180;
  config.l2 = 180;
  config.m1 = 200;
  config.m2 = 200;
  config.n = 220;
  config.p = cv::Point2d(0, 0);
  config.q = cv::Point2d(0, 50);
  config.trim0 = 0;
  config.trim1 = 0;
  return config;
}

void ArmConfig::Encode(double* x) const {
  x[0] = p.x;
  x[1] = p.y;
  x[2] = q.x;
  x[3] = q.y;
  x[4] = l1;
  x[5] = l2;
  x[6] = m1;
  x[7] = m2;
  x[8] = n;
  x[9] = a1;
  x[10] = trim0;
  x[11] = trim1;
}

void ArmConfig::Decode(const double x[12]) {
  p.x = x[0];
  p.y = x[1];
  q.x = x[2];
  q.y = x[3];
  l1 = x[4];
  l2 = x[5];
  m1 = x[6];
  m2 = x[7];
  n = x[8];
  a1 = x[9];
  trim0 = x[10];
  trim1 = x[11];
}

void CalibrationData::Save(const std::string& filename) const {
  std::ofstream ofs(filename);
  for (const auto& p : data) {
    ofs << p.x << " " << p.y << " " << p.angle[0] << " " << p.angle[1]
        << std::endl;
  }
  ofs.close();
}

void CalibrationData::Load(const std::string& filename) {
  std::ifstream ifs(filename);
  data.erase(data.begin(), data.end());
  while (true) {
    CalibrationDataPoint p;
    ifs >> p.x >> p.y >> p.angle[0] >> p.angle[1];
    if (ifs.eof()) {
      break;
    }
    data.push_back(p);
  }
  ifs.close();
}

void CalibrationData::Dump() const {
  for (auto& p : data) {
    std::cerr << p.x << " " << p.y << " " << p.angle[0] << " " << p.angle[1]
              << std::endl;
  }
}

int Optimize(CalibrationData& pts, const ArmConfig& init, ArmConfig* output) {
  const int nFunctions = pts.data.size() * 2;
  const int nVars = 12;  // TODO: embed DoF information in ArmConfig
  double x[nVars];
  init.Encode(x);
  double fvec[nFunctions];
  double tol = 0.0001;
  int iwa[nVars];
  // workarea
  const int lwa = nFunctions * nVars + 5 * nVars + nFunctions;
  double wa[lwa];
  int result = lmdif1(fcn, &pts, nFunctions, nVars, x, fvec, tol, iwa, wa, lwa);
  output->Decode(x);
  return result;
}

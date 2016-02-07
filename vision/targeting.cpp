#include "targeting.h"
#include "v4l_webcam.h"

namespace team254 {

const double kMountAngleDeg = 40.0;  // degrees from horizontal (positive up)
const double kMountAngleRad = kMountAngleDeg * M_PI / 180;
const double kMountHeight = 20.0;  // inches from floor to lens

const double kCenterOfTargetHeight = 89.0;  // inches from floor
const double kDifferentialHeight = kCenterOfTargetHeight - kMountHeight;

const double kCenterRow = kRowsPixels / 2 - .5;
const double kCenterCol = kColsPixels / 2 - .5;

const double kFocalLengthMeters = .00367;
const double kImagerWidthMeters = .0048;
const double kFocalLengthPixels =
    (kColsPixels / kImagerWidthMeters) * kFocalLengthMeters;

std::vector<TargetInfo> getTargetInfo(
    const std::vector<std::pair<double, double>>& detections) {
  std::vector<TargetInfo> rv;
  for (auto& point : detections) {
    // Local camera frame (right handed)
    // +x is away from the imager
    // +y is to the left (-col)
    // +z is up (-row)
    double x = 1.0;
    double y = -(point.first - kCenterCol) / kFocalLengthPixels;
    double z = -(point.second - kCenterRow) / kFocalLengthPixels;

    // rotate by -camera mount angle (about y axis)
    double sin_angle = std::sin(-kMountAngleRad);
    double cos_angle = std::cos(-kMountAngleRad);
    double xr = z * sin_angle + x * cos_angle;
    double yr = y;
    double zr = z * cos_angle - x * sin_angle;

    // find intersection with the goal
    if (zr > 0) {
      double scaling = kDifferentialHeight / zr;
      TargetInfo target;
      target.distance = xr * scaling;
      target.theta = std::atan2(yr, xr) * 180.0 / M_PI;
      rv.push_back(std::move(target));
    }
  }
  return rv;
}

std::string toJSON(const TargetInfo& target) {
  std::stringstream ss;
  ss << "{\"theta\":" << target.theta << ",\"distance\":" << target.distance
     << "}";
  return ss.str();
}

}  // namespace team254

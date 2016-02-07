#include <string>
#include <vector>

namespace team254 {

struct TargetInfo {
  double theta;     // degrees
  double distance;  // inches
};

std::vector<TargetInfo> getTargetInfo(
    const std::vector<std::pair<double, double>>& detections);

std::string toJSON(const TargetInfo& target);

}  // namespace team254

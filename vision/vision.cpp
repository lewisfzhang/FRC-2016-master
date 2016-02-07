#include <sstream>
#include "targeting.h"
#include "udp_client.hpp"
#include "v4l_webcam.h"

using team254::TargetInfo;
using team254::V4LWebcam;

class App {
 public:
  App();
  void run();
};

int main(int argc, char** argv) {
  try {
    App app;
    app.run();
  } catch (const cv::Exception& e) {
    return std::cout << "error: " << e.what() << std::endl, 1;
  } catch (...) {
    return std::cout << "unknown exception" << std::endl, 1;
  }
  return 0;
}

App::App() { cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice()); }

void App::run() {
  const bool kPrintTiming = true;
  const bool kShowVis = false;

  UDPClient client("roborio-254-frc.local", 5254);
  client.connect();

  V4LWebcam webcam("/dev/video0");
  webcam.Configure();
  if (kShowVis) {
    cv::namedWindow("opencv_webcam", cv::WINDOW_NORMAL);
  }
  cv::Mat frame_hsv;
  cv::Mat frame_threshold;
  cv::Mat frame_morphology;
  cv::Mat frame_morphology2;
  cv::Mat frame_vis;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Point> convex_contour;
  std::vector<cv::Point> poly;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::pair<double, double>> target_centers;
  std::vector<TargetInfo> targets;

  webcam.StartStream();
  std::chrono::steady_clock::time_point last_frame_time =
      std::chrono::steady_clock::time_point::min();
  while (true) {
    contours.clear();
    hierarchy.clear();
    targets.clear();
    target_centers.clear();

    auto processing_started = std::chrono::steady_clock::now();
    auto decoded = webcam.DecodeLatestFrame();
    auto decoding_finished = std::chrono::steady_clock::now();
    cv::Mat& frame = decoded.second;
    if (frame.rows == 0 || frame.cols == 0) {
      if (kShowVis) {
        cv::waitKey(1);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      continue;
    }

    cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv, cv::Scalar(57, 231, 44), cv::Scalar(81, 255, 205),
                frame_threshold);
    cv::morphologyEx(frame_threshold, frame_morphology, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)),
                     cv::Point(-1, -1), 1);
    cv::morphologyEx(frame_morphology, frame_morphology2, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)),
                     cv::Point(-1, -1), 1);
    if (kShowVis) {
      frame_vis = frame_morphology2.clone();
    }
    // Filter the image based on shape
    cv::findContours(frame_morphology2, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_TC89_KCOS);

    for (auto& contour : contours) {
      convex_contour.clear();
      cv::convexHull(contour, convex_contour, false);
      poly.clear();
      cv::approxPolyDP(convex_contour, poly, 20, true);
      if (poly.size() == 4 && cv::isContourConvex(poly)) {
        auto moments = cv::moments(poly);
        target_centers.emplace_back(moments.m10 / moments.m00,
                                    moments.m01 / moments.m00);
      }
    }

    std::ostringstream json;
    json << "{\"targets\":[";
    targets = team254::getTargetInfo(target_centers);
    for (auto& target : targets) {
      std::cout << "Found target theta=" << target.theta
                << ", distance=" << target.distance << std::endl;
      json << team254::toJSON(target) << ",";
    }
    json << "]";
    last_frame_time = decoded.first;
    auto processing_done = std::chrono::steady_clock::now();
    json << ", \"capturedAgoMs\": "
         << std::chrono::duration<double, std::milli>(processing_done -
                                                      last_frame_time)
                .count()
         << "}";
    client.send(json.str());

    if (kPrintTiming) {
      std::cout << "Processing start to end: "
                << std::chrono::duration<double, std::milli>(processing_done -
                                                             processing_started)
                       .count()
                << " ms" << std::endl;
      std::cout << "Image stamp to end: "
                << std::chrono::duration<double, std::milli>(processing_done -
                                                             decoded.first)
                       .count()
                << " ms" << std::endl;
      std::cout << "Decoding finished to end: "
                << std::chrono::duration<double, std::milli>(processing_done -
                                                             decoding_finished)
                       .count()
                << " ms" << std::endl;
    }
    if (kShowVis) {
      cv::imshow("opencv_webcam", frame_vis);
      cv::waitKey(1);
    }
  };
}

#include "v4l_webcam.h"

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
  V4LWebcam webcam("/dev/video0");
  webcam.Configure();

  cv::namedWindow("opencv_webcam", cv::WINDOW_NORMAL);
  cv::Mat frame_hsv;
  cv::Mat frame_threshold;
  cv::Mat frame_morphology;
  cv::Mat frame_morphology2;

  webcam.StartStream();
  while (true) {
    cv::Mat frame = webcam.DecodeLatestFrame().second;
    cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv, cv::Scalar(52, 117, 76), cv::Scalar(101, 255, 255),
                frame_threshold);
    cv::morphologyEx(frame_threshold, frame_morphology, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)),
                     cv::Point(-1, -1), 1);
    cv::morphologyEx(frame_morphology, frame_morphology2, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)),
                     cv::Point(-1, -1), 1);

    std::cout << "Frame is " << frame.cols << " x " << frame.rows << std::endl;
    cv::imshow("opencv_webcam", frame_morphology2);
    cv::waitKey(1);
  };
}

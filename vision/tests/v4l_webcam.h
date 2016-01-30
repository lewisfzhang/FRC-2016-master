#include <atomic>
#include <iostream>
#include <string>
#include <thread>
#include <iomanip>
#include <stdexcept>
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>

#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"

const int kRowsPixels = 1080;
const int kColsPixels = 1920;
const int kNumBuffers = 16;

namespace team254 {

class V4LWebcam {
 public:
  V4LWebcam(const std::string& device);
  ~V4LWebcam();

  void Configure(bool calibration = false);

  void StartStream();

  void StopStream();

  cv::Mat DecodeLatestFrame();

 private:
  // State
  bool is_calibration_;
  bool is_configured_;
  std::atomic<bool> stop_streaming_;
  std::thread streaming_thread_;

  std::mutex buffer_bookkeeping_mutex_;
  // Which buffer is locked for use by a processing operation
  int processing_buffer_;  // GUARDED BY buffer_bookkeeping_mutex_
  // Which buffer has been most recently written
  int latest_capture_buffer_;  // GUARDED BY buffer_bookkeeping_mutex_

  // V4L stuff
  const std::string device_;
  int descriptor_;
  std::array<void*, kNumBuffers> capture_buffers_;
  size_t buffer_length_;

  void SetCameraSettings(const v4l2_control& control);
  bool SetAndCheckCameraSettings(const v4l2_control& control);
};

}

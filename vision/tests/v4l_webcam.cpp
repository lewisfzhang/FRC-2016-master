#include "v4l_webcam.h"

namespace team254 {

V4LWebcam::V4LWebcam(const std::string& device)
    : is_calibration_(false),
      is_configured_(false),
      stop_streaming_(false),
      processing_buffer_(-1),
      latest_capture_buffer_(-1),
      device_(device) {
  static_assert(kNumBuffers > 1, "Must have at least 2 capture buffers");
  descriptor_ = v4l2_open(device_.c_str(), O_RDWR);
  if (descriptor_ < 0) {
    perror("v4l2_open");
    exit(1);
  }

  // Make sure the device supports video streaming
  v4l2_capability capabilities;
  if (ioctl(descriptor_, VIDIOC_QUERYCAP, &capabilities) < 0) {
    perror("VIDIOC_QUERYCAP");
    exit(1);
  }
  if (!(capabilities.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
      !(capabilities.capabilities & V4L2_CAP_STREAMING)) {
    std::cerr << "Device does not support video/streaming" << std::endl;
    exit(1);
  }
}

V4LWebcam::~V4LWebcam() {
  if (streaming_thread_) {
    StopStream();
    streaming_thread_.reset();
  }
  if (is_configured_) {
    for (size_t i = 0; i < kNumBuffers; ++i) {
      munmap(capture_buffers_[i], buffer_length_);
    }
  }
  v4l2_close(descriptor_);
}

void V4LWebcam::Configure() {
  is_configured_ = true;
  if (streaming_thread_) {
    std::cout << "Capture thread running; cannot configure camera" << std::endl;
  }

  // set format
  v4l2_format format;
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  format.fmt.pix.width = kColsPixels;
  format.fmt.pix.height = kRowsPixels;

  if (ioctl(descriptor_, VIDIOC_S_FMT, &format) < 0) {
    perror("VIDIOC_S_FMT");
    exit(1);
  }

  // set up buffers
  v4l2_requestbuffers bufrequest;
  bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufrequest.memory = V4L2_MEMORY_MMAP;
  bufrequest.count = kNumBuffers;

  if (ioctl(descriptor_, VIDIOC_REQBUFS, &bufrequest) < 0) {
    perror("VIDIOC_REQUFS");
    exit(1);
  }
  if (bufrequest.count != kNumBuffers) {
    std::cerr << "Asked for " << kNumBuffers << " but only got "
              << bufrequest.count << std::endl;
    exit(1);
  }

  for (size_t i = 0; i < kNumBuffers; ++i) {
    // allocate buffers
    v4l2_buffer bufferinfo;
    memset(&bufferinfo, 0, sizeof(bufferinfo));

    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = i;

    if (ioctl(descriptor_, VIDIOC_QUERYBUF, &bufferinfo) < 0) {
      perror("VIDIOC_QUERYBUF");
      exit(1);
    }
    capture_buffers_[i] = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE,
                               MAP_SHARED, descriptor_, bufferinfo.m.offset);
    capture_times_[i] = TimePoint::min();

    if (capture_buffers_[i] == MAP_FAILED) {
      std::cerr << "Could not set map buffer" << std::endl;
      exit(1);
    }
    memset(capture_buffers_[i], 0, bufferinfo.length);
    buffer_length_ = bufferinfo.length;
  }
}

void V4LWebcam::StartStream(bool calibration) {
  if (!streaming_thread_) {
    is_calibration_ = calibration;
    // Activate streaming
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(descriptor_, VIDIOC_STREAMON, &type) < 0) {
      std::cerr << "Could not start streaming" << std::endl;
      return;
    }

    // For some reason, the c920 camera tends to reset exposure upon beginning
    // streaming
    // See: https://patchwork.linuxtv.org/patch/22822/
    // As a workaround, force the ioctl update here rather than before beginning
    // streaming.
    LoadSettings();

    streaming_thread_.reset(new std::thread([this]() mutable {
      v4l2_buffer bufferinfo;
      memset(&bufferinfo, 0, sizeof(bufferinfo));
      bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      bufferinfo.memory = V4L2_MEMORY_MMAP;
      bufferinfo.index = 0;
      while (!stop_streaming_) {
        {
          // Figure out which buffer to read into.
          std::lock_guard<std::mutex> lock(buffer_bookkeeping_mutex_);
          if (processing_buffer_ == static_cast<int>(bufferinfo.index)) {
            std::cerr << "Capture buffer wrapped around" << std::endl;
            bufferinfo.index = (bufferinfo.index + 1) % kNumBuffers;
          }
          if (latest_capture_buffer_ == static_cast<int>(bufferinfo.index)) {
            // If we have wrapped around to the latest capture buffer, first
            // invalidate it.
            latest_capture_buffer_ = -1;
          }
        }

        // Enqueue the buffer
        // std::cout << "Enqueue buffer " << bufferinfo.index << std::endl;
        // const auto capture_start_time = std::chrono::steady_clock::now();
        if (ioctl(descriptor_, VIDIOC_QBUF, &bufferinfo) < 0) {
          perror("VIDIOC_QBUF");
          exit(1);
        }

        // Blocks until the buffer has been dequeued
        if (ioctl(descriptor_, VIDIOC_DQBUF, &bufferinfo) < 0) {
          perror("VIDIOC_QBUF");
          exit(1);
        }
        // std::cout << "Dequeue buffer " << bufferinfo.index << std::endl;
        const auto capture_end_time = std::chrono::steady_clock::now();
        {
          // Update the latest buffer.
          std::lock_guard<std::mutex> lock(buffer_bookkeeping_mutex_);
          latest_capture_buffer_ = bufferinfo.index;
          capture_times_[bufferinfo.index] = capture_end_time;
        }
        bufferinfo.index = (bufferinfo.index + 1) % kNumBuffers;
      }

      // Stop streaming
      if (ioctl(descriptor_, VIDIOC_STREAMOFF, &bufferinfo.type) < 0) {
        perror("VIDIOC_STREAMOFF");
        exit(1);
      }
      stop_streaming_ = false;
    }));
  }
}

void V4LWebcam::StopStream() {
  if (streaming_thread_) {
    stop_streaming_ = true;
    streaming_thread_->join();
    streaming_thread_.reset();
  }
}

std::pair<TimePoint, cv::Mat> V4LWebcam::DecodeLatestFrame() {
  std::pair<TimePoint, cv::Mat> rv =
      std::make_pair(TimePoint::min(), cv::Mat());
  int buffer = -1;
  {
    std::lock_guard<std::mutex> lock(buffer_bookkeeping_mutex_);
    if (processing_buffer_ != -1 || latest_capture_buffer_ == -1) {
      // std::cerr << "Can only decode one frame at a time" << std::endl;
      return rv;
    }
    // Lock the processing buffer
    processing_buffer_ = latest_capture_buffer_;
    buffer = processing_buffer_;
    rv.first = capture_times_[buffer];
  }
  // std::cout << "Decoding frame from buffer " << buffer << std::endl;
  rv.second = cv::imdecode(
      cv::Mat(kRowsPixels, kColsPixels, CV_8UC3, capture_buffers_[buffer]),
      CV_LOAD_IMAGE_COLOR);
  {
    // Release the processing buffer
    std::lock_guard<std::mutex> lock(buffer_bookkeeping_mutex_);
    processing_buffer_ = -1;
  }
  return rv;
}

void V4LWebcam::SetCameraSettings(const v4l2_control& control) {
  if (v4l2_ioctl(descriptor_, VIDIOC_S_CTRL, &control) != 0) {
    std::cerr << "Could not set property" << std::endl;
  } else {
    std::cout << "Setting property " << control.id << std::endl;
  }
}

void V4LWebcam::LoadSettings() {
  // set manual white balance
  v4l2_control c;
  c.id = V4L2_CID_AUTO_WHITE_BALANCE;
  c.value = 0;
  SetCameraSettings(c);

  // set color temperature
  c.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
  c.value = 4000;
  SetCameraSettings(c);

  // set brightness
  c.id = V4L2_CID_BRIGHTNESS;
  c.value = 20;
  SetCameraSettings(c);

  // set contrast
  c.id = V4L2_CID_CONTRAST;
  c.value = 128;
  SetCameraSettings(c);

  // set saturation
  c.id = V4L2_CID_SATURATION;
  c.value = 255;
  SetCameraSettings(c);

  // set gain
  c.id = V4L2_CID_GAIN;
  c.value = 20;
  SetCameraSettings(c);

  // set sharpness
  c.id = V4L2_CID_SHARPNESS;
  c.value = 255;
  SetCameraSettings(c);

  // set manual focus
  c.id = V4L2_CID_FOCUS_AUTO;
  c.value = 0;
  SetCameraSettings(c);

  // set focus
  c.id = V4L2_CID_FOCUS_ABSOLUTE;
  c.value = 0;
  SetCameraSettings(c);

  // manual exposure control
  c.id = V4L2_CID_EXPOSURE_AUTO;
  c.value = (is_calibration_ ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL);
  SetCameraSettings(c);

  // set exposure time
  c.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  c.value = 10;
  SetCameraSettings(c);
}

}  // namespace team254

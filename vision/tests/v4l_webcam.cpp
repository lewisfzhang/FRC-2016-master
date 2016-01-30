#include "v4l_webcam.h"

namespace team254 {

V4LWebcam::V4LWebcam(const std::string& device)
    : is_calibration_(false),
      is_configured_(false),
      stop_streaming_(false),
      processing_buffer_(-1),
      latest_capture_buffer_(-1),
      device_(device) {
  descriptor_ = v4l2_open(device_, O_RDWR);
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
  }
  if (is_configured_) {
    for (size_t i = 0; i < kNumBuffers; ++i) {
      munmap(capture_buffers_[i], buffer_length_);
    }
  }
  v4l2_close(descriptor_);
}

void V4LWebcam::Configure(bool calibration = false) {
  is_configured_ = true;
  if (streaming_thread_) {
    std::cout << "Capture thread running; cannot configure camera";
  }
  is_calibration_ = calibration;
  v4l2_control c;

  // set manual white balance
  c.id = V4L2_CID_AUTO_WHITE_BALANCE;
  c.value = 0;
  SetAndCheckCameraSettings(c);

  // set color temperature
  c.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
  c.value = 4000;
  SetAndCheckCameraSettings(c);

  // manual exposure control
  c.id = V4L2_CID_EXPOSURE_AUTO;
  c.value = (calibration ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL);
  SetAndCheckCameraSettings(c);

  // set exposure time
  c.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  c.value = 10;
  SetAndCheckCameraSettings(c);

  // set manual focus
  c.id = V4L2_CID_FOCUS_AUTO;
  c.value = 0;
  SetAndCheckCameraSettings(c);

  // set focus
  c.id = V4L2_CID_FOCUS_ABSOLUTE;
  c.value = 0;
  SetAndCheckCameraSettings(c);

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
              << bufrequest.count;
    exit(1);
  }

  // allocate buffers
  v4l_buffer bufferinfo;
  memset(&bufferinfo, 0, sizeof(bufferinfo));

  bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufferinfo.memory = V4L2_MEMORY_MMAP;
  bufferinfo.index = 0;

  if (ioctl(descriptor_, VIDIOC_QUERYBUF, &bufferinfo) < 0) {
    perror("VIDIOC_QUERYBUF");
    exit(1);
  }

  buffer_length_ = bufferinfo.length;
  for (size_t i = 0; i < kNumBuffers; ++i) {
    capture_buffers_[i] = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE,
                               MAP_SHARED, descriptor_, bufferinfo.m.offset);
    ++bufferinfo.index;

    if (capture_buffers_[i] == MAP_FAILED) {
      std::cerr << "Could not set map buffer" << std::endl;
      exit(1);
    }
    memset(capture_buffers_[i], 0, bufferinfo.length);
  }
}

void V4LWebcam::StartStream() {
  if (!streaming_thread_) {
    // Activate streaming
    v4l_buffer bufferinfo;
    memset(&bufferinfo, 0, sizeof(bufferinfo));
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MAP;
    bufferinfo.index = 0;
    int type = bufferinfo.type;
    if (ioctl(descriptor, VIDIOC_STREAMON, &type) < 0) {
      std::cerr << "Could not start streaming" << std::endl;
      return;
    }

    // For some reason, the c920 camera tends to reset exposure upon beginning
    // streaming
    // See: https://patchwork.linuxtv.org/patch/22822/
    // As a workaround, force the ioctl update here.
    // exposure control
    c.id = V4L2_CID_EXPOSURE_AUTO;
    c.value = (is_calibration_ ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL_);
    SetCameraSettings(c, descriptor);

    // set exposure time
    c.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    c.value = 10;
    SetCameraSettings(c, descriptor);

    streaming_thread_ = [this, bufferinfo]() {
      while (!stop_streaming_) {
        {
          // Figure out which buffer to read into.
          std::lock_guard<std::mutex> lock(buffer_bookkeeping_mutex_);
          if (processing_buffer_ == bufferinfo.index) {
            bufferinfo.index = (bufferinfo.index + 1) % kNumBuffers;
            std::cerr << "Capture buffer wrapped around";
          }
        }

        // Enqueue the buffer
        if (ioctl(descriptor_, VIDIOC_QBUF, &bufferinfo) < 0) {
          perror("VIDIOC_QBUF");
          exit(1);
        }

        // Blocks until the buffer has been dequeued
        if (ioctl(descriptor_, VIDIOC_DQBUF, &bufferinfo) < 0) {
          perror("VIDIOC_QBUF");
          exit(1);
        }

        {
          // Update the latest buffer.
          std::lock_guard<std::mutex> lock(buffer_bookkeeping_mutex_);
          latest_capture_buffer_ = bufferinfo.index;
        }
        bufferinfo.index = (bufferinfo.index + 1) % kNumBuffers;
      }

      // Stop streaming
      if (ioctl(descriptor_, VIDIOC_STREAMOFF, &bufferinfo.type) < 0) {
        perror("VIDIOC_STREAMOFF");
        exit(1);
      }
      stop_streaming_ = false;
    };
  }
}

void V4LWebcam::StopStream() {
  if (streaming_thread_) {
    stop_streaming_ = true;
    streaming_thread_.join();
  }
}

cv::Mat V4LWebcam::DecodeLatestFrame() {
  cv::Mat decoded;
  int buffer = -1;
  {
    std::lock_guard<std::mutex> lock(buffer_bookkeeping_mutex_);
    if (processing_buffer_ == -1) {
      std::cerr << "Can only decode one frame at a time" << std::endl;
      return decoded;
    }
    // Lock the processing buffer
    processing_buffer_ = latest_capture_buffer_;
    buffer = processing_buffer_;
  }
  decoded = cv::imdecode(
      cv::Mat(kRowsPixels, kColsPixels, CV_8UC3, capture_buffers_[buffer]),
      CV_LOAD_IMAGE_COLOR);
  {
    // Release the processing buffer
    std::lock_guard<std::mutex> lock(buffer_bookkeeping_mutex_);
    processing_buffer_ = -1;
  }

  return decoded;
}

void V4LWebcam::SetCameraSettings(const v4l2_control& control) {
  if (v4l2_ioctl(descriptor_, VIDIOC_S_CTRL, &control) != 0) {
    std::cerr << "Could not set property" << std::endl;
  } else {
    std::cout << "Setting property " << control.id << std::endl;
  }
}

bool V4LWebcam::SetAndCheckCameraSettings(const v4l2_control& control) {
  int num_retries_left = 10;
  v4l2_control c;
  c.id = control.id;
  c.value = control.value;
  while (num_retries_left > 0) {
    if (v4l2_ioctl(descriptor_, VIDIOC_G_CTRL, &c) == 0) {
      if (c.value != control.value) {
        SetCameraSettings(control);
      } else {
        std::cout << "Property " << c.id << " already at the desired value"
                  << std::endl;
        return true;
      }
    } else {
      std::cerr << "Could not get property" << std::endl;
      return false;
    }
    num_retries_left--;
    std::cout << "Checking property " << c.id << " again..." << std::endl;
  }
  exit(1);
  return false;
}

}  // namespace team254

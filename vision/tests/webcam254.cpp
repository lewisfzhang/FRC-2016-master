#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <iomanip>
#include <stdexcept>
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>

class App {
 public:
  App();
  void run();

 private:
  bool running_;
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

App::App() : running_(false) {
  cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());
}

void SetCameraSettings(const v4l2_control& control, int v4l_descriptor) {
  if (v4l2_ioctl(v4l_descriptor, VIDIOC_S_CTRL, &control) != 0) {
    std::cerr << "Could not set property" << std::endl;
  } else {
    std::cout << "Setting property " << control.id << std::endl;
  }
}

bool SetAndCheckCameraSettings(const v4l2_control& control,
                               int v4l_descriptor) {
  int num_retries_left = 10;
  v4l2_control c;
  c.id = control.id;
  c.value = control.value;
  while (num_retries_left > 0) {
    if (v4l2_ioctl(v4l_descriptor, VIDIOC_G_CTRL, &c) == 0) {
      if (c.value != control.value) {
        SetCameraSettings(control, v4l_descriptor);
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

void App::run() {
  running_ = true;

  // open capture
  int descriptor = v4l2_open("/dev/video0", O_RDWR);
  if (descriptor < 0) {
    std::cerr << "Could not open device";
    return;
    ;
  }

  v4l2_control c;

  // set manual white balance (need to do this before changing exposure for some
  // reason)
  c.id = V4L2_CID_AUTO_WHITE_BALANCE;
  c.value = 0;
  SetAndCheckCameraSettings(c, descriptor);

  // set color temperature
  c.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
  c.value = 4000;
  SetAndCheckCameraSettings(c, descriptor);

  // manual exposure control
  c.id = V4L2_CID_EXPOSURE_AUTO;
  c.value = V4L2_EXPOSURE_MANUAL;
  SetAndCheckCameraSettings(c, descriptor);

  // set exposure time
  c.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  c.value = 10;
  SetAndCheckCameraSettings(c, descriptor);
Parameters:

  // set manual focus
  c.id = V4L2_CID_FOCUS_AUTO;
  c.value = 0;
  SetAndCheckCameraSettings(c, descriptor);

  // set focus
  c.id = V4L2_CID_FOCUS_ABSOLUTE;
  c.value = 0;
  SetAndCheckCameraSettings(c, descriptor);

  v4l2_capability capabilities;
  if (ioctl(descriptor, VIDIOC_QUERYCAP, &capabilities) < 0) {
    perror("VIDIOC;_QUERYCAP");
    exit(1);
  }
  if (!(capabilities.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
      !(capabilities.capabilities & V4L2_CAP_STREAMING)) {
    std::cerr << "Device does not support video/streaming" << std::endl;
    return;
  }

  // set format
  v4l2_format format;
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  format.fmt.pix.width = 1920;
  format.fmt.pix.height = 1080;

  if (ioctl(descriptor, VIDIOC_S_FMT, &format) < 0) {
    perror("VIDIOC_S_FMT");
    return;
  }

  // set up buffers
  v4l2_requestbuffers bufrequest;
  bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufrequest.memory = V4L2_MEMORY_MMAP;
  bufrequest.count = 1;

  if (ioctl(descriptor, VIDIOC_REQBUFS, &bufrequest) < 0) {
    perror("VIDIOC_REQUFS");
    return;
  }

  // allocate buffers
  v4l2_buffer bufferinfo;
  memset(&bufferinfo, 0, sizeof(bufferinfo));

  bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufferinfo.memory = V4L2_MEMORY_MMAP;
  bufferinfo.index = 0;

  if (ioctl(descriptor, VIDIOC_QUERYBUF, &bufferinfo) < 0) {
    perror("VIDIOC_QUERYBUF");
    return;
  }

  void* buffer_start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE,
                            MAP_SHARED, descriptor, bufferinfo.m.offset);

  if (buffer_start == MAP_FAILED) {
    std::cerr << "Could not set map buffer" << std::endl;
    return;
  }
  memset(buffer_start, 0, bufferinfo.length);

  memset(&bufferinfo, 0, sizeof(bufferinfo));

  bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufferinfo.memory = V4L2_MEMORY_MMAP;
  bufferinfo.index = 0;

  // Activate streaming
  int type = bufferinfo.type;
  if (ioctl(descriptor, VIDIOC_STREAMON, &type) < 0) {
    std::cerr << "Could not start streaming" << std::endl;
    return;
  }

  // v4l2_close(descriptor);

  // cv::VideoCapture vc(0);
  cv::Mat frame;
  // vc >> frame;

  // For some reason, the c920 camera tends to reset exposure upon beginning
  // streaming
  // See: https://patchwork.linuxtv.org/patch/22822/
  // As a workaround, force the ioctl update here.
  // manual exposure control
  c.id = V4L2_CID_EXPOSURE_AUTO;
  c.value = V4L2_EXPOSURE_MANUAL;
  SetCameraSettings(c, descriptor);

  // set exposure time
  c.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  c.value = 10;
  SetCameraSettings(c, descriptor);

  // capture loop

  cv::namedWindow("opencv_webcam", cv::WINDOW_NORMAL);
  cv::Mat frame_hsv;
  cv::Mat frame_threshold;
  cv::Mat frame_morphology;
  cv::Mat frame_morphology2;
  cv::Scalar blue(0, 0, 255);
  cv::Scalar green(0, 255, 0);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  // std::vector<int> hull;
  while (running_) {
    cv::waitKey(1);

    // Put the buffer in the incoming queue.
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    if (ioctl(descriptor, VIDIOC_QBUF, &bufferinfo) < 0) {
      perror("VIDIOC_QBUF");
      exit(1);
    }

    // The buffer's waiting in the outgoing queue.
    if (ioctl(descriptor, VIDIOC_DQBUF, &bufferinfo) < 0) {
      perror("VIDIOC_QBUF");
      exit(1);
    }
    frame = cv::imdecode(cv::Mat(1080, 1920, CV_8UC3, buffer_start),
                         CV_LOAD_IMAGE_COLOR);

    // vc >> frame;
    cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);

    // Define range of blue color in HS
    cv::inRange(frame_hsv, cv::Scalar(52, 117, 76), cv::Scalar(101, 255, 255),
                frame_threshold);

    // Morphology (erode + dilate)
    cv::morphologyEx(
        frame_threshold, frame_morphology, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15)),
        cv::Point(-1, -1), 1);
    cv::morphologyEx(
        frame_morphology, frame_morphology2, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15)),
        cv::Point(-1, -1), 1);

    // Contours (finding edges)
    cv::findContours(frame_morphology2, contours, hierarchy, cv::RETR_TREE,
                     cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    std::vector<cv::Moments> mu(contours.size());
    // For testing purposes
    std::cout << "Number of contours: " << contours.size() << std::endl;
    std::vector<std::vector<cv::Point>> hull(contours.size());
    /*
        for (int i = 0; i < contours.size(); i++) {
          // Find convex hull- Outer frame- of the result from findContours()
          cv::convexHull(cv::Mat(contours[i]), hull[i], true);

          // Find moments- Average of image pixel intensities, used to find
       center
          // of mass
          cv::moments(contours[i], false);
          cv::Point centerOfMass =
              (moment['m10'] / moment['m00'], moment['m01'] / moment['m00']);
          cv::drawContours(frame_morphology2, hull, -1, blue, 3, 8, hierarchy,
       0,
                           cv::Point());
        }
        std::cout << "\n\n" << std::endl;
    */
    // INSERT OPENCVhierarchy CALLS HERE
    std::cout << "Frame is " << frame.cols << " x " << frame.rows << std::endl;
    cv::imshow("opencv_webcam", frame_morphology2);
  }

  if (ioctl(descriptor, VIDIOC_STREAMOFF, &type) < 0) {
    perror("VIDIOC_STREAMOFF");
    exit(1);
  }
  v4l2_close(descriptor);
}

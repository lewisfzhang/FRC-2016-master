#include <iostream>
#include <string>
#include <iomanip>
#include <stdexcept>
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>

class App
{
public:
    App();
    void run();

private:
    bool running_;
};

int main(int argc, char** argv)
{
    try
    {
        App app;
        app.run();
    }
    catch (const cv::Exception& e) { return std::cout << "error: "  << e.what() << std::endl, 1; }
    catch(...) { return std::cout << "unknown exception" << std::endl, 1; }
    return 0;
}

App::App() : running_(false)
{
    cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());
}


void App::run()
{
    running_ = true;

    // open capture
    int descriptor = v4l2_open("/dev/video0", O_RDWR);

    // set manual white balance (need to do this before changing exposure for some reason)
    v4l2_control c;
    c.id = V4L2_CID_AUTO_WHITE_BALANCE;
    c.value = 0;
    if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0) {
        std::cout << "Set manual white balance" << std::endl;
    }

    // set color temperature
    c.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
    c.value = 4000;
    if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0) {
        std::cout << "Set white balance temperature to " << c.value << std::endl;
    }

    // manual exposure control
    c.id = V4L2_CID_EXPOSURE_AUTO;
    c.value = V4L2_EXPOSURE_MANUAL;
    if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0) {
        std::cout << "Set manual exposure" << std::endl;
    }

    // set exposure time
    c.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    c.value = 3;
    if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0) {
        std::cout << "Set exposure time to (100us *) " << c.value << std::endl;
    }

    // set manual focus
    c.id = V4L2_CID_FOCUS_AUTO;
    c.value = 0;
    if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0) {
        std::cout << "Set manual focus" << std::endl;
    }

    // set focus
    c.id = V4L2_CID_FOCUS_ABSOLUTE;
    c.value = 0;
    if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0) {
        std::cout << "Set focus to " << c.value << std::endl;
    }

    cv::VideoCapture vc;
    cv::Mat frame;
    vc.open(0);
    cv::namedWindow("opencv_webcam", cv::WINDOW_NORMAL);
    if (!vc.isOpened()) {
        std::cerr << "Could not open camera" << std::endl;
        return;
    }
    while (running_)
    {
        vc >> frame;

// INSERT OPENCV CALLS FROM PYTHON HERE

        std::cout << "Frame is " << frame.cols << " x " << frame.rows << std::endl;
        cv::imshow("opencv_webcam", frame);

        cv::waitKey(1);
    }
}

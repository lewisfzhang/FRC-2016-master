#!/bin/bash
g++ -std=c++11 -O3 v4l_webcam.cpp vision.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_calib3d -lopencv_gpu -lv4l2 -o vision

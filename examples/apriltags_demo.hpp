#ifndef     _APRILTAGS_DEMO_HPP
#define     _APRILTAGS_DEMO_HPP

// --------------------------------------------------------------------------------------------
// Brazilian Institute of Robotics
// Thomio Watanabe
// September 2015
// Description: I took the original code and made some modifications, basically
//              to rewrite it in cpp style 
// --------------------------------------------------------------------------------------------


#include <iostream>
#include <cstring>
#include <vector>
#include <sys/time.h>

#ifndef __APPLE__
    #define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
    #include <libv4l2.h>
    #include <linux/videodev2.h>
    #include <fcntl.h>
    #include <errno.h>
#endif

// OpenCV library for easy access to USB camera and drawing of images on screen
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include <apriltags/TagDetector.h>
#include <apriltags/Tag16h5.h>
#include <apriltags/Tag25h7.h>
#include <apriltags/Tag25h9.h>
#include <apriltags/Tag36h9.h>
#include <apriltags/Tag36h11.h>

// Needed for getopt / command line options processing
#include <unistd.h>

extern int optind;
extern char *optarg;

// For Arduino: locally defined serial port access class
#include "Serial.h"
#include <cmath>



class Demo {
        AprilTags::TagDetector* m_tagDetector;
        AprilTags::TagCodes m_tagCodes;

        bool m_draw; // draw image and April tag detections?
        bool m_arduino; // send tag detections to serial port?

        int m_width; // image size in pixels
        int m_height;
        double m_tagSize; // April tag side length in meters of square black frame
        double m_fx; // camera focal length in pixels
        double m_fy;
        double m_px; // camera principal point
        double m_py;

        int m_deviceId; // camera id (in case of multiple cameras)
        cv::VideoCapture m_cap;

        int m_exposure;
        int m_gain;
        int m_brightness;

        Serial m_serial;

        // utility function to provide current system time (used below in
        // determining frame rate at which images are being processed)
        double tic();
        // changing the tag family
        void setTagCodes(string s);
        double standardRad(double t);
        void wRo_to_euler(Eigen::Matrix3d const& wRo, double& yaw, double& pitch, double& roll);
        void print_detection(AprilTags::TagDetection& detection);

    public:
        // default constructor
        Demo();

        // parse command line options to change default behavior
        void parseOptions(int argc, char* argv[]);

        void setup();

        // The processing loop where images are retrieved, tags detected,
        // and information about detections generated
        void loop();
}; // Demo

#endif

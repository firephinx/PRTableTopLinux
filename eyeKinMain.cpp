/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include <iostream>
#include <signal.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
#include "viewer.h"
#endif

#include "objectSegmentor.h"
#include "calib.h"
#include "utilities.h"

bool protonect_shutdown = false;

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

//The following demostrates how to create a custom logger
#include <fstream>
#include <cstdlib>
class MyFileLogger: public libfreenect2::Logger
{
private:
  std::ofstream logfile_;
public:
  MyFileLogger(const char *filename)
  {
    if (filename)
      logfile_.open(filename);
    level_ = Debug;
  }
  bool good()
  {
    return logfile_.is_open() && logfile_.good();
  }
  virtual void log(Level level, const std::string &message)
  {
    logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
  }
};

/**
 * Main application entry point.
 *
 * Accepted argumemnts:
 * - cpu Perform depth processing with the CPU.
 * - gl  Perform depth processing with OpenGL.
 * - cl  Perform depth processing with OpenCL.
 * - <number> Serial number of the device to open.
 * - -noviewer Disable viewer window.
 */
int main(int argc, char *argv[])
{
  std::string program_path(argv[0]);
  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";

  if(executable_name_idx != std::string::npos)
  {
    binpath = program_path.substr(0, executable_name_idx);
  }

  libfreenect2::Freenect2 freenect2;
  // create a console logger with debug level (default is console logger with info level)
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
  MyFileLogger *filelogger = new MyFileLogger(getenv("LOGFILE"));
  if (filelogger->good())
    libfreenect2::setGlobalLogger(filelogger);

  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return -1;
  }

  std::string serial = freenect2.getDefaultDeviceSerialNumber();

  bool viewer_enabled = true;

  for(int argI = 1; argI < argc; ++argI)
  {
    const std::string arg(argv[argI]);

    if(arg == "cpu")
    {
      if(!pipeline)
        pipeline = new libfreenect2::CpuPacketPipeline();
    }
    else if(arg == "gl")
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
      std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg == "cl")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
      std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg.find_first_not_of("0123456789") == std::string::npos) //check if parameter could be a serial number
    {
      serial = arg;
    }
    else if(arg == "-noviewer")
    {
      viewer_enabled = false;
    }
    else
    {
      std::cout << "Unknown argument: " << arg << std::endl;
    }
  }

  if(pipeline)
  {
    dev = freenect2.openDevice(serial, pipeline);
  }
  else
  {
    dev = freenect2.openDevice(serial);
  }

  if(dev == 0)
  {
    std::cout << "failure opening device!" << std::endl;
    return -1;
  }

  signal(SIGINT,sigint_handler);
  protonect_shutdown = false;

  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

  size_t framecount = 0;
#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
  Viewer viewer;
  if (viewer_enabled)
    viewer.initialize();
#else
  viewer_enabled = false;
#endif

  // Eyekin calibration function calls
  listener.waitForNewFrame(frames);

  cv::Mat calibrgb, calibdepth;
  libfreenect2::Frame *calibrgbFrame = frames[libfreenect2::Frame::Color];
  libfreenect2::Frame *calibdepthFrame = frames[libfreenect2::Frame::Depth];
  cv::Mat(calibrgbFrame->height, calibrgbFrame->width, CV_32FC1, calibrgbFrame->data).copyTo(calibrgb);
  cv::Mat(calibdepthFrame->height, calibdepthFrame->width, CV_32FC1, calibdepthFrame->data).copyTo(calibdepth);

  // Initializes a Calibration object
  personalRobotics::Calib::Calib calib(calibrgb, calibdepth, calibrgb->width, calibrgb->height, dev->getColorCameraParams());
  calib.createLookup();

  listener.release(frames);

  // Checks to make sure everything is calibrated before moving on to the object segmentation code.
  while(!calib.isCalibrated())
  {
    calib.calibrate();
    if(!calib.isCalibrated())
    {
      listener.waitForNewFrame(frames);
      *calibrgbFrame = frames[libfreenect2::Frame::Color];
      *calibdepthFrame = frames[libfreenect2::Frame::Depth];
      cv::Mat(calibrgbFrame->height, calibrgbFrame->width, CV_32FC1, calibrgbFrame->data).copyTo(calibrgb);
      cv::Mat(calibdepthFrame->height, calibdepthFrame->width, CV_32FC1, calibdepthFrame->data).copyTo(calibdepth)
      calib.inputNewFrames(calibrgb, calibdepth);
      listener.release(frames);
    }
  }

  // Accesses the homography and planePtr from the calibration object
  cv::Mat homography = calib.getHomography();
  pcl::ModelCoefficients::Ptr planePtr = calib.getPlanePtr();

  // Initializes an ObjectSegmentor object
  personalRobotics::ObjectSegmentor::ObjectSegmentor OS;
  OS.setPlaneCoefficients(planePtr);
  OS.setHomography(homography);

  //Main Segmentation Loop waiting for new frames
  while(!protonect_shutdown)
  {
    cv::Mat rgb, depth;
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgbFrame = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depthFrame = frames[libfreenect2::Frame::Depth];
    cv::Mat(rgbFrame->height, rgbFrame->width, CV_32FC1, rgbFrame->data).copyTo(rgb);
    cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data).copyTo(depth);

    registration->apply(rgbFrame, depthFrame, &undistorted, &registered);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBPC(new pcl::PointCloud<pcl::PointXYZRGB>);
    personalRobotics::Calib::createCloud(depth, rgb, *RGBPC);

    OS.segment(rgb, RGBPC);

    framecount++;
    if (!viewer_enabled)
    {
      if (framecount % 100 == 0)
        std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
      listener.release(frames);
      continue;
    }

#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
    viewer.addFrame("RGB", rgbFrame);
    viewer.addFrame("depth", depthFrame);
    viewer.addFrame("registered", &registered);

    protonect_shutdown = protonect_shutdown || viewer.render();
#endif

    listener.release(frames);
    //libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));
  }

  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
  dev->stop();
  dev->close();

  delete registration;

  return 0;
}

// -*- C++ -*-
#ifndef CAMERA_PORT_H
#define CAMERA_PORT_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/DataInPort.h>

#include <cnoid/corba/CameraImage.hh>
#include <opencv2/opencv.hpp>
#include <zbar.h>


class CameraPort : public RTC::InPort<Img::TimedCameraImage>
{
public:
  CameraPort(const char *name);

  bool read();


  Img::TimedCameraImage m_image;
  cv::Mat m_frame, m_gray;
  zbar::Image m_zbar;
};



#endif
